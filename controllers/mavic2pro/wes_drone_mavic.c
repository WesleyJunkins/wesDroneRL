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
 * Description:  Simplistic drone control:
 * - Stabilize the robot using the embedded sensors.
 * - Use PID technique to stabilize the drone roll/pitch/yaw.
 * - Use a cubic function applied on the vertical difference to stabilize the robot vertically.
 * - Stabilize the camera.
 * - Control the robot using the computer keyboard.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

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
  // WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");  // Not used in this example.

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
  printf("Start the drone...\n");

  // Wait one second.
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  // Display manual control message.
  printf("You can control the drone with your computer keyboard:\n");
  printf("- 'up': move forward.\n");
  printf("- 'down': move backward.\n");
  printf("- 'right': turn right.\n");
  printf("- 'left': turn left.\n");
  printf("- 'shift + up': increase the target altitude.\n");
  printf("- 'shift + down': decrease the target altitude.\n");
  printf("- 'shift + right': strafe right.\n");
  printf("- 'shift + left': strafe left.\n");

  // Constants, empirically found.
  const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
  const double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  //const double k_vertical_p = 3.0;        // P constant of the vertical PID.
  const double k_roll_p = 50.0;           // P constant of the roll PID.
  const double k_pitch_p = 30.0;          // P constant of the pitch PID.

  // Variables.
  double target_altitude = 1.0;  // The target altitude. Can be changed by the user.
  double target_x = 0.0;
  double target_z = 0.0;
  double target_yaw = -1;
  double kp_x = 10;
  double kd_x = 10;
  double kp_z = 10;
  double kd_z = 10;
  double kp_altitude = 3;
  double kd_altitude = 10;
  double kp_yaw = 1;
  double kd_yaw = 1;
  double error_x = 0;
  double last_error_x = 0;
  double d_error_x = 0;
  double error_z = 0;
  double last_error_z = 0;
  double d_error_z = 0;
  double error_altitude = 0;
  double last_error_altitude = 0;
  double d_error_altitude = 0;
  double error_yaw = 0;
  double last_error_yaw = 0;
  double d_error_yaw = 0;
  double pitch_input = 0;
  double roll_input = 0;
  double vertical_input = 0;
  double position_x = 0;
  double altitude = 0;
  double position_z = 0;

  // Main loop
  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();  // in seconds.

    // Retrieve robot position using the sensors.
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0] + M_PI / 2.0;
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double yaw = *wb_compass_get_values(compass);
    
    position_x = wb_gps_get_values(gps)[0];
    altitude = wb_gps_get_values(gps)[1];
    position_z = wb_gps_get_values(gps)[2];
    
    const double roll_acceleration = wb_gyro_get_values(gyro)[0];
    const double pitch_acceleration = wb_gyro_get_values(gyro)[1];
    
    // const double altitude = wb_gps_get_values(gps)[2];
    // const double roll_velocity = wb_gyro_get_values(gyro)[0];
    // const double pitch_velocity = wb_gyro_get_values(gyro)[1];

    // Blink the front LEDs alternatively with a 1 second rate.
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_acceleration);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_acceleration);

    // Transform the keyboard input to disturbances on the stabilization algorithm.
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;
    
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          pitch_disturbance -= 0.5;
          break;
        case WB_KEYBOARD_DOWN:
          pitch_disturbance += 0.5;
          break;
        case WB_KEYBOARD_RIGHT:
          roll_disturbance -= 0.5;
          break;
        case WB_KEYBOARD_LEFT:
          roll_disturbance += 0.5;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
          target_x -= 0.01;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
          target_x += 0.01;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          target_z -= 0.01;
          //printf("target altitude: %f [m]\n", target_altitude);
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          target_z += 0.01;
          //printf("target altitude: %f [m]\n", target_altitude);
          break;
      }
      key = wb_keyboard_get_key();
    }
    
    // PD Control for x Position
    error_x = CLAMP(target_x + position_x, -1.0, 1.0);
    d_error_x = error_x - last_error_x;
    last_error_x = error_x;
    pitch_input = (k_pitch_p * CLAMP(pitch, -1.0, 1.0)) - 5*pitch_acceleration + pitch_disturbance;
    pitch_input = pitch_input + (kp_x*error_x) + (kd_x*d_error_x);
    
    // PD Control for z Position
    error_z = CLAMP(target_z - position_z, -1.0, 1.0);
    d_error_z = error_z - last_error_z;
    last_error_z = error_z;
    roll_input = (k_roll_p * CLAMP(roll, -1.0, 1.0)) + 5*roll_acceleration + roll_disturbance;
    roll_input = roll_input + (kp_z*error_z) + (kd_z*d_error_z);
    
    // PD Control for altitude Position
    error_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    d_error_altitude = error_altitude - last_error_altitude;
    last_error_altitude = error_altitude;
    vertical_input = (kp_altitude*error_altitude) + (kd_altitude*d_error_altitude);
    
    // PD Control for Rotation
    error_yaw = target_yaw - yaw;
    d_error_yaw = error_yaw - last_error_yaw;
    const double yaw_input = kp_yaw*CLAMP(error_yaw, -1.0, 1.0) + kd_yaw*d_error_yaw + yaw_disturbance;
    last_error_yaw = error_yaw;

    // Compute the roll, pitch, yaw and vertical inputs.
    // const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    // const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    // const double yaw_input = yaw_disturbance;
    // const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    // const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    // Actuate the motors taking into consideration all the computed inputs.
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
