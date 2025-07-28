/*
 * Simplified always-on PID drone controller:
 * - Receives UDP JSON commands {"movement": X, "yaw": Y}
 * - Adds received values to current setpoints for pitch and yaw
 * - PID always active, no stabilizer-off logic
 * - Keyboard input for manual testing
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

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
  wb_keyboard_enable(timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  for (int m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  printf("Start the drone...\n");
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  printf("You can control the drone with your computer keyboard or UDP JSON:\n");
  printf("- 'up': move forward.\n");
  printf("- 'down': move backward.\n");
  printf("- 'right': turn right.\n");
  printf("- 'left': turn left.\n");
  printf("- 'shift + up': increase the target altitude.\n");
  printf("- 'shift + down': decrease the target altitude.\n");
  printf("- 'shift + right': strafe right.\n");
  printf("- 'shift + left': strafe left.\n");
  printf("- UDP JSON: {\"movement\": X, \"yaw\": Y}\n");

  // Constants
  const double k_vertical_thrust = 68.5;
  const double k_vertical_offset = 0.6;
  const double k_vertical_p = 3.0;
  const double k_roll_p = 50.0;
  const double k_pitch_p = 30.0;
  const double k_yaw_p = 10.0;

  double target_altitude = 1.0;
  double pitch_setpoint = 0.0;
  double yaw_setpoint = 0.0;

  // UDP socket setup
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    perror("Socket creation failed");
    return EXIT_FAILURE;
  }
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(SOCKET_PORT);
  server_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    perror("Socket bind failed");
    return EXIT_FAILURE;
  }
  int flags = fcntl(sockfd, F_GETFL, 0);
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

  // Main loop
  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    const double altitude = wb_gps_get_values(gps)[2];
    const double roll_velocity = wb_gyro_get_values(gyro)[0];
    const double pitch_velocity = wb_gyro_get_values(gyro)[1];

    // LEDs
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    // Camera stabilization
    double cam_roll_pos = CLAMP(-0.115 * roll_velocity, -0.5, 0.5);
    double cam_pitch_pos = CLAMP(-0.1 * pitch_velocity, -0.5, 0.5);
    wb_motor_set_position(camera_roll_motor, cam_roll_pos);
    wb_motor_set_position(camera_pitch_motor, cam_pitch_pos);

    // UDP JSON command reception
    char buffer[1024];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int bytes_received = recvfrom(sockfd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT, (struct sockaddr*)&client_addr, &client_len);
    if (bytes_received > 0) {
      buffer[bytes_received] = '\0';
      // Simple JSON parsing for {"movement": X, "yaw": Y}
      char* movement_ptr = strstr(buffer, "\"movement\":");
      char* yaw_ptr = strstr(buffer, "\"yaw\":");
      double movement = 0.0, yaw_delta = 0.0;
      if (movement_ptr) {
        movement_ptr += 11;
        movement = atof(movement_ptr);
      }
      if (yaw_ptr) {
        yaw_ptr += 6;
        yaw_delta = atof(yaw_ptr);
      }
      // Always add received values to setpoints
      pitch_setpoint += movement;
      yaw_setpoint += yaw_delta;
      if (movement != 0.0 || yaw_delta != 0.0) {
        printf("[UDP] movement: %.3f, yaw: %.3f | pitch_setpoint: %.3f, yaw_setpoint: %.3f\n", movement, yaw_delta, pitch_setpoint, yaw_setpoint);
      }
    }

    // Keyboard input (optional, for manual testing)
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          pitch_setpoint -= 2.0;
          printf("[KEY] pitch_setpoint: %.3f\n", pitch_setpoint);
          break;
        case WB_KEYBOARD_DOWN:
          pitch_setpoint += 2.0;
          printf("[KEY] pitch_setpoint: %.3f\n", pitch_setpoint);
          break;
        case WB_KEYBOARD_RIGHT:
          yaw_setpoint -= 1.3;
          printf("[KEY] yaw_setpoint: %.3f\n", yaw_setpoint);
          break;
        case WB_KEYBOARD_LEFT:
          yaw_setpoint += 1.3;
          printf("[KEY] yaw_setpoint: %.3f\n", yaw_setpoint);
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
          roll_disturbance = -1.0;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
          roll_disturbance = 1.0;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          target_altitude += 0.05;
          printf("target altitude: %f [m]\n", target_altitude);
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          target_altitude -= 0.05;
          printf("target altitude: %f [m]\n", target_altitude);
          break;
      }
      key = wb_keyboard_get_key();
    }

    // Compute the roll, pitch, yaw and vertical inputs.
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch - pitch_setpoint, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    const double yaw_input = k_yaw_p * CLAMP(yaw - yaw_setpoint, -1.0, 1.0) + yaw_disturbance;
    const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    // Actuate the motors
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);

    // Debugging print statements (restored)
    printf(
      "[MOTORS] FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f | "
      "roll: %.2f, pitch: %.2f, yaw: %.2f, alt: %.2f | "
      "roll_in: %.2f, pitch_in: %.2f, vert_in: %.2f, yaw_in: %.2f | "
      "pitch_set: %.2f, yaw_set: %.2f\n",
      front_left_motor_input, front_right_motor_input, rear_left_motor_input, rear_right_motor_input,
      roll, pitch, yaw, altitude,
      roll_input, pitch_input, vertical_input, yaw_input,
      pitch_setpoint, yaw_setpoint
    );
  }

  close(sockfd);
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
// WES