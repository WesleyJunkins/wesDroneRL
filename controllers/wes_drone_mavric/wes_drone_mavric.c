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
  
  // Initialize PID controllers with more aggressive parameters for early correction
  struct PID position_x_pid = {
    .Kp = 2.0,           // Increased proportional gain for faster response
    .Ki = 0.05,          // Increased integral gain for better steady-state
    .Kd = 0.3,           // Increased derivative gain for better damping
    .Kaw = 0.1,          // Anti-windup gain
    .T_C = 0.01,         // Derivative filter time constant
    .T = timestep / 1000.0, // Time step in seconds
    .max = 3.0,          // Increased max command for stronger corrections
    .min = -3.0,         // Increased min command for stronger corrections
    .max_rate = 2.0,     // Increased max rate for faster response
    .integral = 0.0,
    .err_prev = 0.0,
    .deriv_prev = 0.0,
    .command_sat_prev = 0.0,
    .command_prev = 0.0
  };
  
  struct PID position_y_pid = {
    .Kp = 2.0,           // Increased proportional gain for faster response
    .Ki = 0.05,          // Increased integral gain for better steady-state
    .Kd = 0.3,           // Increased derivative gain for better damping
    .Kaw = 0.1,          // Anti-windup gain
    .T_C = 0.01,         // Derivative filter time constant
    .T = timestep / 1000.0, // Time step in seconds
    .max = 3.0,          // Increased max command for stronger corrections
    .min = -3.0,         // Increased min command for stronger corrections
    .max_rate = 2.0,     // Increased max rate for faster response
    .integral = 0.0,
    .err_prev = 0.0,
    .deriv_prev = 0.0,
    .command_sat_prev = 0.0,
    .command_prev = 0.0
  };
  
  struct PID yaw_pid = {
    .Kp = 2.0,           // Increased proportional gain for faster response
    .Ki = 0.05,          // Increased integral gain for better steady-state
    .Kd = 0.3,           // Increased derivative gain for better damping
    .Kaw = 0.1,          // Anti-windup gain
    .T_C = 0.01,         // Derivative filter time constant
    .T = timestep / 1000.0, // Time step in seconds
    .max = 2.0,          // Increased max command for stronger corrections
    .min = -2.0,         // Increased min command for stronger corrections
    .max_rate = 1.0,     // Increased max rate for faster response
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
  
  // Set socket to non-blocking mode
  int flags = fcntl(sockfd, F_GETFL, 0);
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
  
  // Variables for acceleration tracking
  double prev_velocity_x = 0.0;
  double prev_velocity_y = 0.0;
  double prev_velocity_z = 0.0;
  double dt = timestep / 1000.0;  // Time step in seconds
  
  // Variables for smooth setpoint updates
  static bool was_controlled = false;
  static bool setpoint_updated = false;
  const double acceleration_threshold = 0.1;  // m/s² threshold for "nearly stopped"
  
  // Acceleration limits to prevent continuous buildup
  const double max_disturbance = 2.0;  // Maximum disturbance value
  static double current_roll_disturbance = 0.0;
  static double current_pitch_disturbance = 0.0;
  static double current_yaw_disturbance = 0.0;
  const double disturbance_ramp_rate = 0.5;  // How fast disturbances can increase per timestep

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

    // Calculate acceleration from velocity changes
    double current_velocity_x = (current_x - prev_velocity_x) / dt;
    double current_velocity_y = (current_y - prev_velocity_y) / dt;
    double current_velocity_z = (altitude - prev_velocity_z) / dt;
    
    double acceleration_x = (current_velocity_x - prev_velocity_x) / dt;
    double acceleration_y = (current_velocity_y - prev_velocity_y) / dt;
    double acceleration_z = (current_velocity_z - prev_velocity_z) / dt;
    
    // Clamp acceleration values to prevent infinite values in JSON
    acceleration_x = CLAMP(acceleration_x, -100.0, 100.0);
    acceleration_y = CLAMP(acceleration_y, -100.0, 100.0);
    acceleration_z = CLAMP(acceleration_z, -100.0, 100.0);
    
    // Update previous velocities for next iteration
    prev_velocity_x = current_velocity_x;
    prev_velocity_y = current_velocity_y;
    prev_velocity_z = current_velocity_z;

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

    // Keyboard control - use disturbances like the original controller with limits
    double target_roll_disturbance = 0.0;
    double target_pitch_disturbance = 0.0;
    double target_yaw_disturbance = 0.0;
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          target_pitch_disturbance = -max_disturbance;
          break;
        case WB_KEYBOARD_DOWN:
          target_pitch_disturbance = max_disturbance;
          break;
        case WB_KEYBOARD_RIGHT:
          target_yaw_disturbance = -max_disturbance;
          break;
        case WB_KEYBOARD_LEFT:
          target_yaw_disturbance = max_disturbance;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
          target_roll_disturbance = -max_disturbance;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
          target_roll_disturbance = max_disturbance;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          // Increase altitude
          target_altitude += altitude_change;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          // Decrease altitude
          target_altitude -= altitude_change;
          break;
      }
      key = wb_keyboard_get_key();
    }
    
    // Ramp disturbances towards target values with limits
    if (target_pitch_disturbance > current_pitch_disturbance) {
      current_pitch_disturbance += disturbance_ramp_rate * dt;
      if (current_pitch_disturbance > target_pitch_disturbance) {
        current_pitch_disturbance = target_pitch_disturbance;
      }
    } else if (target_pitch_disturbance < current_pitch_disturbance) {
      current_pitch_disturbance -= disturbance_ramp_rate * dt;
      if (current_pitch_disturbance < target_pitch_disturbance) {
        current_pitch_disturbance = target_pitch_disturbance;
      }
    }
    
    if (target_roll_disturbance > current_roll_disturbance) {
      current_roll_disturbance += disturbance_ramp_rate * dt;
      if (current_roll_disturbance > target_roll_disturbance) {
        current_roll_disturbance = target_roll_disturbance;
      }
    } else if (target_roll_disturbance < current_roll_disturbance) {
      current_roll_disturbance -= disturbance_ramp_rate * dt;
      if (current_roll_disturbance < target_roll_disturbance) {
        current_roll_disturbance = target_roll_disturbance;
      }
    }
    
    if (target_yaw_disturbance > current_yaw_disturbance) {
      current_yaw_disturbance += disturbance_ramp_rate * dt;
      if (current_yaw_disturbance > target_yaw_disturbance) {
        current_yaw_disturbance = target_yaw_disturbance;
      }
    } else if (target_yaw_disturbance < current_yaw_disturbance) {
      current_yaw_disturbance -= disturbance_ramp_rate * dt;
      if (current_yaw_disturbance < target_yaw_disturbance) {
        current_yaw_disturbance = target_yaw_disturbance;
      }
    }
    
    // Decay disturbances when no keys are pressed
    if (fabs(target_roll_disturbance) < 0.1) {
      current_roll_disturbance *= 0.9;  // Decay to zero
    }
    if (fabs(target_pitch_disturbance) < 0.1) {
      current_pitch_disturbance *= 0.9;  // Decay to zero
    }
    if (fabs(target_yaw_disturbance) < 0.1) {
      current_yaw_disturbance *= 0.9;  // Decay to zero
    }
    
    // Use the limited disturbances
    double roll_disturbance = current_roll_disturbance;
    double pitch_disturbance = current_pitch_disturbance;
    double yaw_disturbance = current_yaw_disturbance;

    // Check if disturbances are nearly zero (drone is not being actively controlled)
    bool disturbances_zeroed = (fabs(roll_disturbance) < 0.1 && 
                               fabs(pitch_disturbance) < 0.1 && 
                               fabs(yaw_disturbance) < 0.1);
    
    // Print disturbance values for debugging
    printf("Disturbances - Roll: %.3f, Pitch: %.3f, Yaw: %.3f | Zeroed: %s | Takeoff complete: %s | Setpoints - X: %.3f, Y: %.3f, Yaw: %.3f\n", 
           roll_disturbance, pitch_disturbance, yaw_disturbance, 
           disturbances_zeroed ? "YES" : "NO", 
           takeoff_complete ? "YES" : "NO",
           setpoint_x, setpoint_y, setpoint_yaw);
    
    // Position stabilization (X,Y) - when disturbances are zeroed OR during takeoff
    // This is separate from altitude, roll, pitch, yaw stabilization which should always be active
    bool should_stabilize_position = position_initialized && 
                                    (disturbances_zeroed || !takeoff_complete);  // Stabilize when not controlled OR during takeoff
    
    // Only apply position correction when conditions are met
    double position_correction_x = 0.0;
    double position_correction_y = 0.0;
    
    if (should_stabilize_position) {
      // PID control for X position (forward/backward)
      position_correction_x = -PID_Step(&position_x_pid, current_x, setpoint_x);
      
      // PID control for Y position (left/right)
      position_correction_y = PID_Step(&position_y_pid, current_y, setpoint_y);
    } else {
      // Reset PID integrators when disturbances are active
      position_x_pid.integral = 0.0;
      position_y_pid.integral = 0.0;
    }
    
    // Update setpoints to current position when control stops (disturbances become zero)
    bool is_controlled = !disturbances_zeroed;
    
    if (was_controlled && !is_controlled && !setpoint_updated && position_initialized) {
      // Control stopped, update setpoints to current position
      setpoint_x = current_x;
      setpoint_y = current_y;
      setpoint_updated = true;
    }
    
    // Reset setpoint update flag when control becomes active again
    if (is_controlled) {
      setpoint_updated = false;
    }
    
    was_controlled = is_controlled;

    // Yaw stabilization - keep heading fixed using PID control
    double yaw_correction = 0.0;
    
    if (yaw_initialized && fabs(yaw_disturbance) < 0.1) {
      // Only apply yaw correction when no yaw disturbance is active
      // Calculate yaw error (handle angle wrapping)
      double yaw_error = yaw - setpoint_yaw;
      
      // Normalize yaw error to [-π, π]
      while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
      while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
      
      // PID control for yaw using normalized error
      yaw_correction = PID_Step(&yaw_pid, yaw, setpoint_yaw);
    } else {
      // Reset yaw PID integrator when disturbance is active
      yaw_pid.integral = 0.0;
    }

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
    // Altitude, roll, pitch, and yaw stabilization are ALWAYS active
    // Position stabilization (X,Y) is only active when conditions are met
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance + position_correction_y;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance + position_correction_x;
    const double yaw_input = yaw_disturbance + yaw_correction; // Yaw stabilization always active

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