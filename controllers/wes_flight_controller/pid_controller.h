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
 * @file pid_controller.h
 * Description: PID controller for Mavic 2 Pro drone
 * Author: Adapted from Crazyflie implementation
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <math.h>

// Actual state structure
typedef struct {
  double roll;
  double pitch;
  double yaw_rate;
  double altitude;
  double vx;
  double vy;
} actual_state_t;

// Desired state structure
typedef struct {
  double roll;
  double pitch;
  double yaw_rate;
  double altitude;
  double vx;
  double vy;
} desired_state_t;

// Motor power structure
typedef struct {
  double m1;
  double m2;
  double m3;
  double m4;
} motor_power_t;

// PID gains structure
typedef struct {
  double kp_att_y;
  double kd_att_y;
  double kp_att_rp;
  double kd_att_rp;
  double kp_vel_xy;
  double kd_vel_xy;
  double kp_z;
  double ki_z;
  double kd_z;
} gains_pid_t;

// PID controller structure
typedef struct {
  double kp;
  double ki;
  double kd;
  double kaw;
  double T_C;
  double T;
  double max;
  double min;
  double max_rate;
  double integral;
  double err_prev;
  double deriv_prev;
  double command_sat_prev;
  double command_prev;
} pid_controller_t;

// Function declarations
void init_pid_attitude_fixed_height_controller(void);
void pid_velocity_fixed_height_controller(actual_state_t actual_state, 
                                        desired_state_t* desired_state, 
                                        gains_pid_t gains_pid, 
                                        double dt, 
                                        motor_power_t* motor_power);

// PID step function
double pid_step(pid_controller_t* pid, double measurement, double setpoint);

#endif // PID_CONTROLLER_H 