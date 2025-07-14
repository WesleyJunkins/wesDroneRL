/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Simple drone takeoff and hover controller:
 * - Take off to 1 meter altitude
 * - Maintain stable hover at 1 meter using sensor feedback
 * - Stabilize roll, pitch, and yaw using PID control
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h> // Required for fcntl

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define SOCKET_PORT 8888
#define SOCKET_IP "127.0.0.1"

// PID Controller structure based on Simone Bertoni's implementation
struct PID {
    float Kp;              // Proportional gain constant
    float Ki;              // Integral gain constant
    float Kd;              // Derivative gain constant
    float Kaw;             // Anti-windup gain constant
    float T_C;             // Time constant for derivative filtering
    float T;               // Time step
    float max;             // Max command
    float min;             // Min command
    float max_rate;        // Max rate of change of the command
    float integral;        // Integral term
    float err_prev;        // Previous error
    float deriv_prev;      // Previous derivative
    float command_sat_prev;// Previous saturated command
    float command_prev;    // Previous command
};

// PID Step function implementation
float PID_Step(struct PID *pid, float measurement, float setpoint) {
    float err;
    float command;
    float command_sat;
    float deriv_filt;

    /* Error calculation */
    err = setpoint - measurement;

    /* Integral term calculation - including anti-windup */
    pid->integral += pid->Ki * err * pid->T + pid->Kaw * (pid->command_sat_prev - pid->command_prev) * pid->T;
    
    /* Derivative term calculation using filtered derivative method */
    deriv_filt = (err - pid->err_prev + pid->T_C * pid->deriv_prev) / (pid->T + pid->T_C);
    pid->err_prev = err;
    pid->deriv_prev = deriv_filt;

    /* Summing the 3 terms */
    command = pid->Kp * err + pid->integral + pid->Kd * deriv_filt;

    /* Remember command at previous step */
    pid->command_prev = command;

    /* Saturate command */
    if (command > pid->max) {
        command_sat = pid->max;
    } else if (command < pid->min) {
        command_sat = pid->min;
    } else {
        command_sat = command;
    }

    /* Apply rate limiter */
    if (command_sat > pid->command_sat_prev + pid->max_rate * pid->T) {
        command_sat = pid->command_sat_prev + pid->max_rate * pid->T;
    } else if (command_sat < pid->command_sat_prev - pid->max_rate * pid->T) {
        command_sat = pid->command_sat_prev - pid->max_rate * pid->T;
    }

    /* Remember saturated command at previous step */
    pid->command_sat_prev = command_sat;

    return command_sat;
}

// Function to create JSON message with drone state
void create_drone_json(char* buffer, size_t buffer_size, 
                      double accel_x, double accel_y, double accel_z,
                      double set_x, double set_y, double set_yaw,
                      double curr_x, double curr_y, double yaw,
                      double roll, double pitch) {
    snprintf(buffer, buffer_size,
        "{"
        "\"acceleration\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
        "\"setpoints\":{\"translation\":{\"x\":%.3f,\"y\":%.3f},\"rotation\":{\"yaw\":%.3f}},"
        "\"current\":{\"translation\":{\"x\":%.3f,\"y\":%.3f},\"rotation\":{\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f}}"
        "}",
        accel_x, accel_y, accel_z,
        set_x, set_y, set_yaw,
        curr_x, curr_y, yaw,
        roll, pitch);
}

// Function to send JSON data via UDP socket
int send_json_data(int sockfd, const char* json_data) {
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SOCKET_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SOCKET_IP);
    
    int bytes_sent = sendto(sockfd, json_data, strlen(json_data), 0,
                           (struct sockaddr*)&server_addr, sizeof(server_addr));
    
    if (bytes_sent < 0) {
        perror("Error sending data");
        return -1;
    }
    return bytes_sent;
}

// Function to receive JSON commands via UDP socket
int receive_json_command(int sockfd, double* movement, double* yaw) {
    char buffer[1024];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    
    int bytes_received = recvfrom(sockfd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT,
                                 (struct sockaddr*)&client_addr, &client_len);
    
    if (bytes_received > 0) {
        buffer[bytes_received] = '\0';  // Null terminate the string
        
        // Simple JSON parsing for movement and yaw values
        // Expected format: {"movement": 0.5, "yaw": -0.3}
        char* movement_ptr = strstr(buffer, "\"movement\":");
        char* yaw_ptr = strstr(buffer, "\"yaw\":");
        
        if (movement_ptr) {
            movement_ptr += 11;  // Skip "movement":
            *movement = atof(movement_ptr);
        }
        
        if (yaw_ptr) {
            yaw_ptr += 6;  // Skip "yaw":
            *yaw = atof(yaw_ptr);
        }
        
        printf("*** RECEIVED COMMAND: movement=%.3f, yaw=%.3f ***\n", *movement, *yaw);
        return bytes_received;
    }
    
    return 0;  // No data received
}

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Get and enable devices.
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
  wb_keyboard_enable(timestep);

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  int m;
  for (m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  // Display the welcome message.
  printf("Starting drone takeoff sequence...\n");

  // Wait one second.
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  // Display keyboard control instructions
  printf("Keyboard Controls:\n");
  printf("- Arrow Keys: Move drone (forward/backward/left/right)\n");
  printf("- Shift + Arrow Keys: Strafe and altitude control\n");
  printf("- Shift + Up/Down: Increase/decrease altitude\n");
  printf("- Shift + Left/Right: Strafe left/right\n");
  printf("Drone will maintain stability while following your commands.\n\n");

  // Control constants
  const double k_vertical_thrust = 68.5;  // Base thrust to lift the drone
  const double k_vertical_offset = 0.6;   // Vertical offset for stabilization
  const double k_vertical_p = 3.0;        // P constant for vertical PID
  const double k_roll_p = 50.0;           // P constant for roll PID
  const double k_pitch_p = 30.0;          // P constant for pitch PID

  // Target altitude and control variables
  double target_altitude = 1.0;     // Target altitude in meters
  const double altitude_threshold = 0.05; // Acceptable error for hovering
  bool takeoff_complete = false;
  
  // Position and rotation setpoints
  double setpoint_x = 0.0;                // Target X position
  double setpoint_y = 0.0;                // Target Y position
  double setpoint_yaw = 0.0;              // Target yaw angle
  
  // Keyboard control constants (using original disturbance values)
  const double movement_speed = 0.1;      // Meters per key press
  const double rotation_speed = 0.1;      // Radians per key press
  const double altitude_change = 0.05;    // Meters per key press
  
  // Position stabilization variables
  double initial_x = 0.0;
  double initial_y = 0.0;
  bool position_initialized = false;
  
  // Yaw stabilization variables
  double initial_yaw = 0.0;
  bool yaw_initialized = false;
  
  // Initialize PID controllers with conservative parameters based on article insights
  // Focus on rate control rather than position control to avoid integral windup
  struct PID position_x_pid = {
    .Kp = 1.0,           // Conservative proportional gain
    .Ki = 0.0,           // NO integral term to prevent windup and circular drift
    .Kd = 0.1,           // Small derivative gain for damping
    .Kaw = 0.0,          // No anti-windup needed without integral
    .T_C = 0.01,         // Derivative filter time constant
    .T = timestep / 1000.0, // Time step in seconds
    .max = 1.5,          // Conservative max command
    .min = -1.5,         // Conservative min command
    .max_rate = 1.0,     // Conservative max rate
    .integral = 0.0,
    .err_prev = 0.0,
    .deriv_prev = 0.0,
    .command_sat_prev = 0.0,
    .command_prev = 0.0
  };
  
  struct PID position_y_pid = {
    .Kp = 1.0,           // Conservative proportional gain
    .Ki = 0.0,           // NO integral term to prevent windup and circular drift
    .Kd = 0.1,           // Small derivative gain for damping
    .Kaw = 0.0,          // No anti-windup needed without integral
    .T_C = 0.01,         // Derivative filter time constant
    .T = timestep / 1000.0, // Time step in seconds
    .max = 1.5,          // Conservative max command
    .min = -1.5,         // Conservative min command
    .max_rate = 1.0,     // Conservative max rate
    .integral = 0.0,
    .err_prev = 0.0,
    .deriv_prev = 0.0,
    .command_sat_prev = 0.0,
    .command_prev = 0.0
  };
  
  struct PID yaw_pid = {
    .Kp = 1.5,           // Conservative proportional gain for yaw
    .Ki = 0.0,           // NO integral term to prevent windup
    .Kd = 0.2,           // Small derivative gain for damping
    .Kaw = 0.0,          // No anti-windup needed without integral
    .T_C = 0.01,         // Derivative filter time constant
    .T = timestep / 1000.0, // Time step in seconds
    .max = 1.0,          // Conservative max command
    .min = -1.0,         // Conservative min command
    .max_rate = 0.5,     // Conservative max rate
    .integral = 0.0,
    .err_prev = 0.0,
    .deriv_prev = 0.0,
    .command_sat_prev = 0.0,
    .command_prev = 0.0
  };

  // Initialize UDP socket for communication
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    perror("Socket creation failed");
    return EXIT_FAILURE;
  }
  
  // Bind socket to listen on the same port
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(SOCKET_PORT);
  server_addr.sin_addr.s_addr = INADDR_ANY;
  
  if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    perror("Socket bind failed");
    return EXIT_FAILURE;
  }
  
  // Set socket to non-blocking mode
  int flags = fcntl(sockfd, F_GETFL, 0);
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
  
  // Variables for velocity and acceleration tracking
  double prev_position_x = 0.0;
  double prev_position_y = 0.0;
  double prev_position_z = 0.0;
  double dt = timestep / 1000.0;  // Time step in seconds
  
  // Variables for smooth setpoint updates
  static bool was_controlled = false;
  static bool setpoint_updated = false;
  const double acceleration_threshold = 0.1;  // m/s² threshold for "nearly stopped"
  
  // Variables for yaw change detection
  static double previous_yaw = 0.0;
  static bool yaw_changed_significantly = false;
  const double yaw_change_threshold = 0.1;  // Radians threshold for significant yaw change
  
  // Variables for settling time and stability detection
  static double settling_start_time = 0.0;
  static bool settling_period_active = false;
  const double settling_time = 1.0;  // Seconds to wait after disturbances stop before updating setpoints
  const double velocity_threshold = 0.05;  // m/s threshold for "nearly stopped"
  
  // Waypoint control system - discrete movement commands
  const double waypoint_distance = 0.5;  // Meters to move per command
  const double waypoint_yaw_change = 0.5;  // Radians to rotate per command (~28.6 degrees)
  static bool waypoint_active = false;
  static double waypoint_target_x = 0.0;
  static double waypoint_target_y = 0.0;
  static double waypoint_target_yaw = 0.0;
  static bool waypoint_reached = false;
  const double waypoint_tolerance = 0.1;  // Meters tolerance for position
  const double yaw_tolerance = 0.1;  // Radians tolerance for yaw
  
  // Command timing system - disable stabilization for 3 seconds after receiving commands
  static double last_command_time = 0.0;
  const double command_timeout = 3.0;  // Seconds to wait after command before re-enabling stabilization
  
  // Persistent movement and yaw command system
  static double current_movement_command = 0.0;  // Current movement velocity (persistent until new command)
  static double current_yaw_command = 0.0;       // Current yaw target (persistent until new command)
  static bool yaw_target_active = false;         // Whether we're actively rotating to a yaw target
  static double yaw_target = 0.0;                // Target yaw angle to rotate to
  const double yaw_target_tolerance = 0.1;       // Radians tolerance for yaw target

  // Main control loop
  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();

    // Read sensor data
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    const double altitude = wb_gps_get_values(gps)[2];
    const double roll_velocity = wb_gyro_get_values(gyro)[0];
    const double pitch_velocity = wb_gyro_get_values(gyro)[1];
    const double current_x = wb_gps_get_values(gps)[0];
    const double current_y = wb_gps_get_values(gps)[1];

    // Calculate velocity from position changes (only after first iteration)
    double current_velocity_x = 0.0;
    double current_velocity_y = 0.0;
    double current_velocity_z = 0.0;
    
    if (position_initialized) {
      current_velocity_x = (current_x - prev_position_x) / dt;
      current_velocity_y = (current_y - prev_position_y) / dt;
      current_velocity_z = (altitude - prev_position_z) / dt;
    }
    
    // Calculate acceleration from velocity changes (simplified)
    double acceleration_x = 0.0;
    double acceleration_y = 0.0;
    double acceleration_z = 0.0;
    
    // Clamp velocity and acceleration values to prevent infinite values
    current_velocity_x = CLAMP(current_velocity_x, -10.0, 10.0);
    current_velocity_y = CLAMP(current_velocity_y, -10.0, 10.0);
    current_velocity_z = CLAMP(current_velocity_z, -10.0, 10.0);
    acceleration_x = CLAMP(acceleration_x, -100.0, 100.0);
    acceleration_y = CLAMP(acceleration_y, -100.0, 100.0);
    acceleration_z = CLAMP(acceleration_z, -100.0, 100.0);
    
    // Update previous positions for next iteration
    prev_position_x = current_x;
    prev_position_y = current_y;
    prev_position_z = altitude;

    // Initialize position reference on first iteration
    if (!position_initialized) {
      initial_x = current_x;
      initial_y = current_y;
      setpoint_x = current_x;  // Set target to current position
      setpoint_y = current_y;  // Set target to current position
      position_initialized = true;
    }
    
    // Initialize yaw reference on first iteration
    if (!yaw_initialized) {
      initial_yaw = yaw;
      setpoint_yaw = yaw;  // Set target to current yaw
      previous_yaw = yaw;  // Initialize previous yaw for change detection
      yaw_initialized = true;
    }

    // Blink LEDs to show the controller is running
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    // Stabilize the camera
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_velocity);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_velocity);

    // Check if takeoff is complete
    if (!takeoff_complete && fabs(altitude - target_altitude) < altitude_threshold) {
      takeoff_complete = true;
    }

    // Receive JSON commands via UDP
    double received_movement = 0.0;
    double received_yaw = 0.0;
    int bytes_received = receive_json_command(sockfd, &received_movement, &received_yaw);
    
    // Process received commands
    if (bytes_received > 0) {
      last_command_time = time;
      printf("*** COMMAND TIMER RESET: %.3f ***\n", last_command_time);
      
      // Process movement command (additive) - only add if not zero
      if (received_movement != 0.0) {
        current_movement_command += received_movement;
        printf("*** MOVEMENT COMMAND: %.3f (total: %.3f) ***\n", received_movement, current_movement_command);
      }
      
      // Process yaw command (additive) - only add if not zero
      if (received_yaw != 0.0) {
        current_yaw_command += received_yaw;
        yaw_target = current_yaw_command;  // Set new yaw target
        yaw_target_active = true;          // Activate yaw targeting
        printf("*** YAW COMMAND: %.3f (total: %.3f) - Target set to %.3f ***\n", received_yaw, current_yaw_command, yaw_target);
      }
    }
    
    // Keyboard control - discrete waypoint commands
    int key = wb_keyboard_get_key();
    while (key > 0) {
      if (!waypoint_active) {  // Only accept new commands if no waypoint is active
        switch (key) {
          case WB_KEYBOARD_UP:
            // Move forward
            waypoint_target_x = current_x + waypoint_distance * cos(yaw);
            waypoint_target_y = current_y + waypoint_distance * sin(yaw);
            waypoint_target_yaw = yaw;  // Keep current yaw
            waypoint_active = true;
            waypoint_reached = false;
            printf("*** WAYPOINT SET: Forward to (%.3f, %.3f, %.3f) ***\n", waypoint_target_x, waypoint_target_y, waypoint_target_yaw);
            break;
          case WB_KEYBOARD_DOWN:
            // Move backward
            waypoint_target_x = current_x - waypoint_distance * cos(yaw);
            waypoint_target_y = current_y - waypoint_distance * sin(yaw);
            waypoint_target_yaw = yaw;  // Keep current yaw
            waypoint_active = true;
            waypoint_reached = false;
            printf("*** WAYPOINT SET: Backward to (%.3f, %.3f, %.3f) ***\n", waypoint_target_x, waypoint_target_y, waypoint_target_yaw);
            break;
          case WB_KEYBOARD_LEFT:
            // Rotate left
            waypoint_target_x = current_x;  // Keep current position
            waypoint_target_y = current_y;
            waypoint_target_yaw = yaw + waypoint_yaw_change;
            waypoint_active = true;
            waypoint_reached = false;
            printf("*** WAYPOINT SET: Rotate left to yaw %.3f ***\n", waypoint_target_yaw);
            break;
          case WB_KEYBOARD_RIGHT:
            // Rotate right
            waypoint_target_x = current_x;  // Keep current position
            waypoint_target_y = current_y;
            waypoint_target_yaw = yaw - waypoint_yaw_change;
            waypoint_active = true;
            waypoint_reached = false;
            printf("*** WAYPOINT SET: Rotate right to yaw %.3f ***\n", waypoint_target_yaw);
            break;
          case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
            // Strafe right
            waypoint_target_x = current_x + waypoint_distance * cos(yaw + M_PI/2);
            waypoint_target_y = current_y + waypoint_distance * sin(yaw + M_PI/2);
            waypoint_target_yaw = yaw;  // Keep current yaw
            waypoint_active = true;
            waypoint_reached = false;
            printf("*** WAYPOINT SET: Strafe right to (%.3f, %.3f, %.3f) ***\n", waypoint_target_x, waypoint_target_y, waypoint_target_yaw);
            break;
          case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
            // Strafe left
            waypoint_target_x = current_x + waypoint_distance * cos(yaw - M_PI/2);
            waypoint_target_y = current_y + waypoint_distance * sin(yaw - M_PI/2);
            waypoint_target_yaw = yaw;  // Keep current yaw
            waypoint_active = true;
            waypoint_reached = false;
            printf("*** WAYPOINT SET: Strafe left to (%.3f, %.3f, %.3f) ***\n", waypoint_target_x, waypoint_target_y, waypoint_target_yaw);
            break;
          case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
            // Increase altitude
            target_altitude += altitude_change;
            printf("*** ALTITUDE SET: %.3f ***\n", target_altitude);
            break;
          case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
            // Decrease altitude
            target_altitude -= altitude_change;
            printf("*** ALTITUDE SET: %.3f ***\n", target_altitude);
            break;
        }
      }
      key = wb_keyboard_get_key();
    }
    
    // Waypoint control system - check if waypoint is reached
    if (waypoint_active && !waypoint_reached) {
      // Calculate distance to waypoint
      double distance_to_waypoint = sqrt(pow(current_x - waypoint_target_x, 2) + pow(current_y - waypoint_target_y, 2));
      
      // Calculate yaw error (handle wrapping)
      double yaw_error = waypoint_target_yaw - yaw;
      while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
      while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
      double yaw_distance = fabs(yaw_error);
      
      // Check if waypoint is reached
      if (distance_to_waypoint < waypoint_tolerance && yaw_distance < yaw_tolerance) {
        waypoint_reached = true;
        printf("*** WAYPOINT REACHED: (%.3f, %.3f, %.3f) ***\n", current_x, current_y, yaw);
      }
    }
    
    // Yaw target system - check if yaw target is reached
    if (yaw_target_active) {
      // Calculate yaw error to target (handle wrapping)
      double yaw_error_to_target = yaw_target - yaw;
      while (yaw_error_to_target > M_PI) yaw_error_to_target -= 2 * M_PI;
      while (yaw_error_to_target < -M_PI) yaw_error_to_target += 2 * M_PI;
      double yaw_distance_to_target = fabs(yaw_error_to_target);
      
      // Check if yaw target is reached
      if (yaw_distance_to_target < yaw_target_tolerance) {
        yaw_target_active = false;
        printf("*** YAW TARGET REACHED: %.3f (current: %.3f) ***\n", yaw_target, yaw);
      }
    }
    
    // Detect significant yaw changes (for debugging)
    double yaw_change = fabs(yaw - previous_yaw);
    // Handle angle wrapping
    if (yaw_change > M_PI) {
      yaw_change = 2 * M_PI - yaw_change;
    }
    
    if (yaw_change > yaw_change_threshold) {
      yaw_changed_significantly = true;
    }
    
    // Waypoint-based control logic - update setpoints when waypoint is reached
    bool waypoint_complete = waypoint_active && waypoint_reached;
    
    // Update setpoints when waypoint is reached
    if (waypoint_complete && !setpoint_updated && position_initialized && yaw_initialized) {
      // Waypoint reached, update setpoints to current position and yaw
      setpoint_x = current_x;
      setpoint_y = current_y;
      setpoint_yaw = yaw;
      setpoint_updated = true;
      
      // Reset waypoint system
      waypoint_active = false;
      waypoint_reached = false;
      
      printf("*** SETPOINTS UPDATED: X=%.3f, Y=%.3f, Yaw=%.3f ***\n", setpoint_x, setpoint_y, setpoint_yaw);
    }
    
    // Check if we're within the command timeout period
    bool command_recently_received = (time - last_command_time) < command_timeout;
    
    // Position stabilization - disabled when waypoint is active OR when command recently received OR when movement command is active
    bool should_stabilize_position = position_initialized && 
                                    (!waypoint_active && !command_recently_received && current_movement_command == 0.0) || !takeoff_complete;  // Stabilize when no waypoint active AND no recent commands AND no movement command OR during takeoff
    
    // Yaw stabilization - disabled when waypoint is active OR when command recently received OR when yaw target is active
    bool should_stabilize_yaw = yaw_initialized && 
                                (!waypoint_active && !command_recently_received && !yaw_target_active) || !takeoff_complete;  // Stabilize when no waypoint active AND no recent commands AND no yaw target active OR during takeoff
    
    // Print waypoint status for debugging
    printf("Waypoint - Active: %s | Reached: %s | Target: (%.3f, %.3f, %.3f) | Current: (%.3f, %.3f, %.3f) | Command Timer: %.1fs | Movement: %.3f | Yaw Target: %s (%.3f) | Stabilizing - Pos: %s, Yaw: %s | Setpoints - X: %.3f, Y: %.3f, Yaw: %.3f\n", 
           waypoint_active ? "YES" : "NO", 
           waypoint_reached ? "YES" : "NO",
           waypoint_target_x, waypoint_target_y, waypoint_target_yaw,
           current_x, current_y, yaw,
           time - last_command_time,
           current_movement_command,
           yaw_target_active ? "YES" : "NO", yaw_target,
           should_stabilize_position ? "YES" : "NO",
           should_stabilize_yaw ? "YES" : "NO",
           setpoint_x, setpoint_y, setpoint_yaw);
    
    // Only apply position correction when conditions are met
    // Use simple proportional control to avoid integral windup issues
    double position_correction_x = 0.0;
    double position_correction_y = 0.0;
    
    if (should_stabilize_position) {
      // Simple proportional control for X position (forward/backward)
      double x_error = setpoint_x - current_x;
      position_correction_x = -position_x_pid.Kp * CLAMP(x_error, -1.0, 1.0);
      
      // Simple proportional control for Y position (left/right)
      double y_error = setpoint_y - current_y;
      position_correction_y = position_y_pid.Kp * CLAMP(y_error, -1.0, 1.0);
    } else if (current_movement_command != 0.0) {
      // Apply movement command - forward/backward movement in drone's frame
      position_correction_x = current_movement_command;
      printf("*** APPLYING MOVEMENT: %.3f ***\n", current_movement_command);
    }
    
    // Yaw stabilization - keep heading fixed using simple proportional control
    double yaw_correction = 0.0;
    
    if (should_stabilize_yaw) {
      // Calculate yaw error (handle angle wrapping)
      double yaw_error = setpoint_yaw - yaw;
      
      // Normalize yaw error to [-π, π]
      while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
      while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
      
      // Simple proportional control for yaw
      yaw_correction = yaw_pid.Kp * CLAMP(yaw_error, -1.0, 1.0);
    } else if (yaw_target_active) {
      // Apply yaw target command - rotate to target yaw
      double yaw_error_to_target = yaw_target - yaw;
      
      // Normalize yaw error to [-π, π]
      while (yaw_error_to_target > M_PI) yaw_error_to_target -= 2 * M_PI;
      while (yaw_error_to_target < -M_PI) yaw_error_to_target += 2 * M_PI;
      
      // Proportional control to reach yaw target
      yaw_correction = yaw_pid.Kp * CLAMP(yaw_error_to_target, -1.0, 1.0);
      printf("*** APPLYING YAW TARGET: %.3f (error: %.3f) ***\n", yaw_target, yaw_error_to_target);
    }
    
    // Reset setpoint update flag when waypoint becomes active
    if (waypoint_active) {
      setpoint_updated = false;
    }
    
    // Update previous yaw for next iteration
    previous_yaw = yaw;

    // Create and send JSON data via socket
    char json_buffer[1024];
    create_drone_json(json_buffer, sizeof(json_buffer),
                     acceleration_x, acceleration_y, acceleration_z,
                     setpoint_x, setpoint_y, setpoint_yaw,
                     current_x, current_y, yaw,
                     roll, pitch);
    
    // Send JSON data (non-blocking)
    int bytes_sent = send_json_data(sockfd, json_buffer);
    if (bytes_sent < 0) {
      // Only print error messages, not success messages
    }

    // Compute control inputs for stabilization
    // Altitude, roll, and pitch stabilization are ALWAYS active
    // Position (X,Y) and yaw stabilization are only active when conditions are met
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + position_correction_y;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + position_correction_x;
    const double yaw_input = yaw_correction; // Yaw stabilization conditional

    // Compute vertical control input (altitude stabilization always active)
    const double altitude_error = target_altitude - altitude + k_vertical_offset;
    const double clamped_altitude_error = CLAMP(altitude_error, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_altitude_error, 3.0);

    // Set motor velocities for all four propellers
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;

    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
  }

  // Cleanup socket
  close(sockfd);
  
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
//WES