/*
 * Copyright 2022 Bitcraze AB
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
 *  ...........       ____  _ __
 *  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 *  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 *  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 *
 * @file wes_flight_controller.c
 * Description: Controls the Mavic 2 Pro drone in webots
 * Author:      Adapted from Crazyflie implementation
 */

#include <math.h>
#include <stdio.h>

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Add external controller
#include "pid_controller.h"

#define FLYING_ALTITUDE 1.0

int main(int argc, char **argv) {
  wb_robot_init();

  const int timestep = (int)wb_robot_get_basic_time_step();

  // Initialize motors for Mavic 2 Pro
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  wb_motor_set_position(front_left_motor, INFINITY);
  wb_motor_set_velocity(front_left_motor, 1.0);
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  wb_motor_set_position(front_right_motor, INFINITY);
  wb_motor_set_velocity(front_right_motor, -1.0);
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  wb_motor_set_position(rear_left_motor, INFINITY);
  wb_motor_set_velocity(rear_left_motor, -1.0);
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  wb_motor_set_position(rear_right_motor, INFINITY);
  wb_motor_set_velocity(rear_right_motor, 1.0);

  // Initialize sensors
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");

  // Wait for 2 seconds
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }

  // Initialize variables
  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};
  double past_x_global = 0;
  double past_y_global = 0;
  double past_time = wb_robot_get_time();

  // Initialize PID gains.
  gains_pid_t gains_pid;
  gains_pid.kp_att_y = 1;
  gains_pid.kd_att_y = 0.5;
  gains_pid.kp_att_rp = 0.5;
  gains_pid.kd_att_rp = 0.1;
  gains_pid.kp_vel_xy = 2;
  gains_pid.kd_vel_xy = 0.5;
  gains_pid.kp_z = 10;
  gains_pid.ki_z = 5;
  gains_pid.kd_z = 5;
  init_pid_attitude_fixed_height_controller();

  double height_desired = FLYING_ALTITUDE;

  // Initialize struct for motor power
  motor_power_t motor_power;

  printf("\n");

  printf("====== Controls =======\n");

  printf(" The Mavic 2 Pro can be controlled from your keyboard!\n");
  printf(" All controllable movement is in body coordinates\n");
  printf("- Use the up, back, right and left button to move in the horizontal plane\n");
  printf("- Use Q and E to rotate around yaw\n ");
  printf("- Use W and S to go up and down\n");

  while (wb_robot_step(timestep) != -1) {
    const double dt = wb_robot_get_time() - past_time;

    // Get measurements
    actual_state.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    actual_state.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    actual_state.yaw_rate = wb_gyro_get_values(gyro)[2];
    actual_state.altitude = wb_gps_get_values(gps)[2];
    double x_global = wb_gps_get_values(gps)[0];
    double vx_global = (x_global - past_x_global) / dt;
    double y_global = wb_gps_get_values(gps)[1];
    double vy_global = (y_global - past_y_global) / dt;

    // Get body fixed velocities
    double actualYaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    double cosyaw = cos(actualYaw);
    double sinyaw = sin(actualYaw);
    actual_state.vx = vx_global * cosyaw + vy_global * sinyaw;
    actual_state.vy = -vx_global * sinyaw + vy_global * cosyaw;

    // Initialize values
    desired_state.roll = 0;
    desired_state.pitch = 0;
    desired_state.vx = 0;
    desired_state.vy = 0;
    desired_state.yaw_rate = 0;
    desired_state.altitude = 1.0;

    double forward_desired = 0;
    double sideways_desired = 0;
    double yaw_desired = 0;
    double height_diff_desired = 0;

    // Control altitude
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          forward_desired = +0.5;
          break;
        case WB_KEYBOARD_DOWN:
          forward_desired = -0.5;
          break;
        case WB_KEYBOARD_RIGHT:
          sideways_desired = -0.5;
          break;
        case WB_KEYBOARD_LEFT:
          sideways_desired = +0.5;
          break;
        case 'Q':
          yaw_desired = 1.0;
          break;
        case 'E':
          yaw_desired = -1.0;
          break;
        case 'W':
          height_diff_desired = 0.1;
          break;
        case 'S':
          height_diff_desired = -0.1;
          break;
      }
      key = wb_keyboard_get_key();
    }

    height_desired += height_diff_desired * dt;

    // Camera stabilization
    double roll_velocity = wb_gyro_get_values(gyro)[0];
    double pitch_velocity = wb_gyro_get_values(gyro)[1];
    double cam_roll_pos = -0.115 * roll_velocity;
    double cam_pitch_pos = -0.1 * pitch_velocity;
    wb_motor_set_position(camera_roll_motor, cam_roll_pos);
    wb_motor_set_position(camera_pitch_motor, cam_pitch_pos);

    desired_state.yaw_rate = yaw_desired;

    // PID velocity controller with fixed height
    desired_state.vy = sideways_desired;
    desired_state.vx = forward_desired;
    desired_state.altitude = height_desired;
    pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motor_power);

    // Setting motorspeed for Mavic 2 Pro
    wb_motor_set_velocity(front_left_motor, motor_power.m1);
    wb_motor_set_velocity(front_right_motor, -motor_power.m2);
    wb_motor_set_velocity(rear_left_motor, -motor_power.m3);
    wb_motor_set_velocity(rear_right_motor, motor_power.m4);

    // Save past time for next time step
    past_time = wb_robot_get_time();
    past_x_global = x_global;
    past_y_global = y_global;
  };

  wb_robot_cleanup();

  return 0;
}