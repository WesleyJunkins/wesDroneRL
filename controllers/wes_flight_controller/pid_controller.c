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
 * @file pid_controller.c
 * Description: PID controller implementation for Mavic 2 Pro drone
 * Author: Adapted from Crazyflie implementation
 */

#include "pid_controller.h"
#include <stdio.h>

// PID controller instances
static pid_controller_t pid_roll;
static pid_controller_t pid_pitch;
static pid_controller_t pid_yaw;
static pid_controller_t pid_altitude;
static pid_controller_t pid_velocity_x;
static pid_controller_t pid_velocity_y;

// PID step function implementation
double pid_step(pid_controller_t* pid, double measurement, double setpoint) {
  double err;
  double command;
  double command_sat;
  double deriv_filt;

  /* Error calculation */
  err = setpoint - measurement;

  /* Integral term calculation - including anti-windup */
  pid->integral += pid->ki * err * pid->T + pid->kaw * (pid->command_sat_prev - pid->command_prev) * pid->T;
  
  /* Derivative term calculation using filtered derivative method */
  deriv_filt = (err - pid->err_prev + pid->T_C * pid->deriv_prev) / (pid->T + pid->T_C);
  pid->err_prev = err;
  pid->deriv_prev = deriv_filt;

  /* Summing the 3 terms */
  command = pid->kp * err + pid->integral + pid->kd * deriv_filt;

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

void init_pid_attitude_fixed_height_controller(void) {
  // Initialize roll PID
  pid_roll.kp = 6.0;
  pid_roll.ki = 3.0;
  pid_roll.kd = 0.0;
  pid_roll.kaw = 0.0;
  pid_roll.T_C = 0.0;
  pid_roll.T = 0.01;
  pid_roll.max = 20.0;
  pid_roll.min = -20.0;
  pid_roll.max_rate = 100.0;
  pid_roll.integral = 0.0;
  pid_roll.err_prev = 0.0;
  pid_roll.deriv_prev = 0.0;
  pid_roll.command_sat_prev = 0.0;
  pid_roll.command_prev = 0.0;

  // Initialize pitch PID
  pid_pitch.kp = 6.0;
  pid_pitch.ki = 3.0;
  pid_pitch.kd = 0.0;
  pid_pitch.kaw = 0.0;
  pid_pitch.T_C = 0.0;
  pid_pitch.T = 0.01;
  pid_pitch.max = 20.0;
  pid_pitch.min = -20.0;
  pid_pitch.max_rate = 100.0;
  pid_pitch.integral = 0.0;
  pid_pitch.err_prev = 0.0;
  pid_pitch.deriv_prev = 0.0;
  pid_pitch.command_sat_prev = 0.0;
  pid_pitch.command_prev = 0.0;

  // Initialize yaw PID
  pid_yaw.kp = 6.0;
  pid_yaw.ki = 1.0;
  pid_yaw.kd = 0.35;
  pid_yaw.kaw = 0.0;
  pid_yaw.T_C = 0.0;
  pid_yaw.T = 0.01;
  pid_yaw.max = 360.0;
  pid_yaw.min = -360.0;
  pid_yaw.max_rate = 100.0;
  pid_yaw.integral = 0.0;
  pid_yaw.err_prev = 0.0;
  pid_yaw.deriv_prev = 0.0;
  pid_yaw.command_sat_prev = 0.0;
  pid_yaw.command_prev = 0.0;

  // Initialize altitude PID
  pid_altitude.kp = 6.0;
  pid_altitude.ki = 3.0;
  pid_altitude.kd = 0.0;
  pid_altitude.kaw = 0.0;
  pid_altitude.T_C = 0.0;
  pid_altitude.T = 0.01;
  pid_altitude.max = 20.0;
  pid_altitude.min = -20.0;
  pid_altitude.max_rate = 100.0;
  pid_altitude.integral = 0.0;
  pid_altitude.err_prev = 0.0;
  pid_altitude.deriv_prev = 0.0;
  pid_altitude.command_sat_prev = 0.0;
  pid_altitude.command_prev = 0.0;

  // Initialize velocity X PID
  pid_velocity_x.kp = 2.0;
  pid_velocity_x.ki = 0.0;
  pid_velocity_x.kd = 0.0;
  pid_velocity_x.kaw = 0.0;
  pid_velocity_x.T_C = 0.0;
  pid_velocity_x.T = 0.01;
  pid_velocity_x.max = 20.0;
  pid_velocity_x.min = -20.0;
  pid_velocity_x.max_rate = 100.0;
  pid_velocity_x.integral = 0.0;
  pid_velocity_x.err_prev = 0.0;
  pid_velocity_x.deriv_prev = 0.0;
  pid_velocity_x.command_sat_prev = 0.0;
  pid_velocity_x.command_prev = 0.0;

  // Initialize velocity Y PID
  pid_velocity_y.kp = 2.0;
  pid_velocity_y.ki = 0.0;
  pid_velocity_y.kd = 0.0;
  pid_velocity_y.kaw = 0.0;
  pid_velocity_y.T_C = 0.0;
  pid_velocity_y.T = 0.01;
  pid_velocity_y.max = 20.0;
  pid_velocity_y.min = -20.0;
  pid_velocity_y.max_rate = 100.0;
  pid_velocity_y.integral = 0.0;
  pid_velocity_y.err_prev = 0.0;
  pid_velocity_y.deriv_prev = 0.0;
  pid_velocity_y.command_sat_prev = 0.0;
  pid_velocity_y.command_prev = 0.0;
}

void pid_velocity_fixed_height_controller(actual_state_t actual_state, 
                                        desired_state_t* desired_state, 
                                        gains_pid_t gains_pid, 
                                        double dt, 
                                        motor_power_t* motor_power) {
  // Update PID time steps
  pid_roll.T = dt;
  pid_pitch.T = dt;
  pid_yaw.T = dt;
  pid_altitude.T = dt;
  pid_velocity_x.T = dt;
  pid_velocity_y.T = dt;

  // Update PID gains from gains_pid structure
  pid_roll.kp = gains_pid.kp_att_rp;
  pid_roll.kd = gains_pid.kd_att_rp;
  pid_pitch.kp = gains_pid.kp_att_rp;
  pid_pitch.kd = gains_pid.kd_att_rp;
  pid_yaw.kp = gains_pid.kp_att_y;
  pid_yaw.kd = gains_pid.kd_att_y;
  pid_altitude.kp = gains_pid.kp_z;
  pid_altitude.ki = gains_pid.ki_z;
  pid_altitude.kd = gains_pid.kd_z;
  pid_velocity_x.kp = gains_pid.kp_vel_xy;
  pid_velocity_x.kd = gains_pid.kd_vel_xy;
  pid_velocity_y.kp = gains_pid.kp_vel_xy;
  pid_velocity_y.kd = gains_pid.kd_vel_xy;

  // Velocity control
  double roll_desired = pid_step(&pid_velocity_y, actual_state.vy, desired_state->vy);
  double pitch_desired = -pid_step(&pid_velocity_x, actual_state.vx, desired_state->vx);

  // Attitude control
  double roll_output = pid_step(&pid_roll, actual_state.roll, roll_desired);
  double pitch_output = pid_step(&pid_pitch, actual_state.pitch, pitch_desired);
  double yaw_output = pid_step(&pid_yaw, actual_state.yaw_rate, desired_state->yaw_rate);
  double altitude_output = pid_step(&pid_altitude, actual_state.altitude, desired_state->altitude);

  // Motor mixing for Mavic 2 Pro (X-frame configuration)
  // Front left, front right, rear left, rear right
  motor_power->m1 = altitude_output + roll_output + pitch_output + yaw_output;  // Front left
  motor_power->m2 = altitude_output - roll_output + pitch_output - yaw_output;  // Front right  
  motor_power->m3 = altitude_output + roll_output - pitch_output - yaw_output;  // Rear left
  motor_power->m4 = altitude_output - roll_output - pitch_output + yaw_output;  // Rear right

  // Apply base thrust (hover thrust)
  double base_thrust = 68.5;  // Mavic 2 Pro hover thrust
  motor_power->m1 += base_thrust;
  motor_power->m2 += base_thrust;
  motor_power->m3 += base_thrust;
  motor_power->m4 += base_thrust;
} 