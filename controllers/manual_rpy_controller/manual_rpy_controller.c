/*
 * Manual RPY Controller for Webots Drone
 * - Keeps drone in the air (altitude hold only)
 * - Allows manual control of roll, pitch, and yaw setpoints via keyboard
 * - Arrow keys: adjust pitch (up/down) and roll (left/right)
 * - A/D: adjust yaw
 * - Shift+Up/Down: adjust altitude
 * - Prints current setpoints and IMU values each loop
 */

#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/led.h>
#include <stdio.h>
#include <math.h>

#define CLAMP(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))

int main() {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Devices
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  for (int m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");

  // Control constants
  const double k_vertical_thrust = 68.5;
  const double k_vertical_offset = 0.6;
  const double k_vertical_p = 3.0;
  const double k_roll_p = 50.0;
  const double k_pitch_p = 30.0;
  const double k_yaw_p = 10.0;

  // Setpoints
  double target_altitude = 1.0;
  double set_roll = 0.0;
  double set_pitch = 0.0;
  double set_yaw = 0.0;

  printf("Manual RPY Controller\n");
  printf("Arrow keys: pitch/roll | A/D: yaw | Shift+Up/Down: altitude\n");

  while (wb_robot_step(timestep) != -1) {
    double time = wb_robot_get_time();
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double roll = rpy[0];
    double pitch = rpy[1];
    double yaw = rpy[2];
    double altitude = wb_gps_get_values(gps)[2];
    double roll_rate = wb_gyro_get_values(gyro)[0];
    double pitch_rate = wb_gyro_get_values(gyro)[1];

    // Blink LEDs
    wb_led_set(front_left_led, ((int)time) % 2);
    wb_led_set(front_right_led, !(((int)time) % 2));

    // Keyboard input for manual RPY
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          set_pitch += 0.05;
          break;
        case WB_KEYBOARD_DOWN:
          set_pitch -= 0.05;
          break;
        case WB_KEYBOARD_LEFT:
          set_roll += 0.05;
          break;
        case WB_KEYBOARD_RIGHT:
          set_roll -= 0.05;
          break;
        case 'A':
        case 'a':
          set_yaw += 0.05;
          break;
        case 'D':
        case 'd':
          set_yaw -= 0.05;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          target_altitude += 0.05;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          target_altitude -= 0.05;
          break;
      }
      key = wb_keyboard_get_key();
    }

    // Altitude hold only
    double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    // Manual RPY control (no stabilization)
    double roll_input = k_roll_p * CLAMP(set_roll, -1.0, 1.0);
    double pitch_input = k_pitch_p * CLAMP(set_pitch, -1.0, 1.0);
    double yaw_input = k_yaw_p * CLAMP(set_yaw, -1.0, 1.0);

    // Motor mixing
    double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;

    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);

    // Print setpoints and IMU
    printf("[SET] roll: %.2f, pitch: %.2f, yaw: %.2f | [IMU] roll: %.2f, pitch: %.2f, yaw: %.2f | alt: %.2f\n",
      set_roll, set_pitch, set_yaw, roll, pitch, yaw, altitude);
  }

  wb_robot_cleanup();
  return 0;
} 