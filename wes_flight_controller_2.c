#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Clamp Helper: assists in limiting the range of a value to a specified range.
#define CLAMP(value, low, high)                                                \
  ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

// Integral Anti-Windup Helper: assists in limiting the range of an integral
// term to a specified range
#define CLAMP_INTEGRAL(i)                                                      \
  (i = CLAMP(i, MIN_INTEGRAL_ANTI_WINDUP, MAX_INTEGRAL_ANTI_WINDUP))

// Tunable Values --------------------------------------------------------------
// -----------------------------------------------------------------------------

// PID Constants (Tunable)
#define KP_ROLL 25.0
#define KI_ROLL 0.0
#define KD_ROLL 3.0

#define KP_PITCH 25.0
#define KI_PITCH 0.0
#define KD_PITCH 3.0

#define KP_YAW 10.0
#define KI_YAW 0.0
#define KD_YAW 1.0

#define KP_Z 40.0
#define KI_Z 0.5
#define KD_Z 15.0

// Motor Constants (Tunable)
#define HOVER_THRUST 55.0
#define MAX_THRUST 100.0
#define MIN_THRUST -100.0

// Integral Anti-Windup Constants (Tunable)
#define MIN_INTEGRAL_ANTI_WINDUP -100.0
#define MAX_INTEGRAL_ANTI_WINDUP 100.0

// Time Constants (Tunable)
#define TUNING_STEPS 1000

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// Tuners ---------------------------------------------------------------------
// -----------------------------------------------------------------------------

// Simulate a test run for given PID constants and return a score (e.g., total
// squared error).
double run_altitude_test(double kp, double ki, double kd, WbDeviceTag imu,
                         WbDeviceTag gps, WbDeviceTag motor) {
  double target_z = 1.0; // Target hover altitude
  double pos_z = 0.0;
  double integral = 0.0, prev_error = 0.0, error = 0.0, derivative = 0.0;
  double total_error = 0.0;
  double dt = 0.01;
  double thrust = HOVER_THRUST;
  int steps = TUNING_STEPS;

  for (int i = 0; i < steps; i++) {
    wb_robot_step((int)(dt * 1000));
    pos_z = wb_gps_get_values(gps)[2];
    error = target_z - pos_z;
    integral += error * dt;
    derivative = (error - prev_error) / dt;
    thrust = CLAMP(HOVER_THRUST + kp * error + ki * integral + kd * derivative,
                   MIN_THRUST, MAX_THRUST);
    wb_motor_set_velocity(motor, thrust);
    total_error += error * error;
    prev_error = error;
  }

  return total_error / steps; // Return average squared error
}

// TWIDDLE OPTIMIZATION
void tune_pid_twiddle(WbDeviceTag imu, WbDeviceTag gps, WbDeviceTag motor) {
  double params[3] = {KP_Z, KI_Z, KD_Z};
  double dp[3] = {1.0, 0.1, 0.5};
  double best_error = run_altitude_test(params[0], params[1], params[2], imu,
                                        gps, motor);
  double tolerance = 0.05;

  printf("Starting Twiddle... Initial error: %.5f\n", best_error);

  while ((dp[0] + dp[1] + dp[2]) > tolerance) {
    for (int i = 0; i < 3; ++i) {
      params[i] += dp[i];
      double error = run_altitude_test(params[0], params[1], params[2], imu,
                                       gps, motor);

      if (error < best_error) {
        best_error = error;
        dp[i] *= 1.1;
      } else {
        params[i] -= 2 * dp[i];
        error = run_altitude_test(params[0], params[1], params[2], imu, gps,
                                  motor);

        if (error < best_error) {
          best_error = error;
          dp[i] *= 1.1;
        } else {
          params[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }

  printf("Twiddle completed. Optimal PID: Kp = %.3f, Ki = %.3f, Kd = %.3f\n",
         params[0], params[1], params[2]);

  printf("// --- Tuned PID Constants ---\n");
  printf("#define KP_Z %.3f\n#define KI_Z %.3f\n#define KD_Z %.3f\n", params[0],
         params[1], params[2]);
}

// ZIEGLER-NICHOLS METHOD
void tune_pid_ziegler_nichols(WbDeviceTag imu, WbDeviceTag gps,
                              WbDeviceTag motor) {
  double Ku = 20.0; // Start with estimated ultimate gain
  double Tu = 0.5;  // Oscillation period, to be measured manually or simulated

  // Use ZN formula for classic PID
  double Kp = 0.6 * Ku;
  double Ki = 2.0 * Kp / Tu;
  double Kd = Kp * Tu / 8.0;

  printf("Ziegler-Nichols PID Constants:\n");
  printf("Ku = %.2f, Tu = %.2f\n", Ku, Tu);
  printf("Kp = %.3f, Ki = %.3f, Kd = %.3f\n", Kp, Ki, Kd);

  // Optional: run a test with them
  double error = run_altitude_test(Kp, Ki, Kd, imu, gps, motor);
  printf("Resulting average error with ZN PID: %.4f\n", error);

  printf("// --- Tuned PID Constants ---\n");
  printf("#define KP_Z %.3f\n#define KI_Z %.3f\n#define KD_Z %.3f\n", Kp, Ki,
         Kd);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

int main(int argc, char **argv) {
  // IMU Variables
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  // Gyroscope Variables
  double roll_rate = 0.0;
  double pitch_rate = 0.0;
  double yaw_rate = 0.0;

  // GPS Variables
  double pos_x = 0.0;
  double pos_y = 0.0;
  double pos_z = 0.0;

  // Target Variables
  double target_roll = 0.0;
  double target_pitch = 0.0;
  double target_yaw = 0.0;
  double target_pos_x = 0.0;
  double target_pos_y = 0.0;
  double target_pos_z = 0.0;
  double hover_thrust = HOVER_THRUST;
  double base_thrust = 0.0;

  // PID Variables
  double error_roll = 0.0;
  double error_pitch = 0.0;
  double error_yaw = 0.0;
  double error_z = 0.0;
  double integral_roll = 0.0;
  double integral_pitch = 0.0;
  double integral_yaw = 0.0;
  double integral_z = 0;
  double derivative_roll = 0;
  double derivative_pitch = 0;
  double derivative_yaw = 0;
  double derivative_z = 0;
  double prev_error_roll = 0.0;
  double prev_error_pitch = 0.0;
  double prev_error_yaw = 0.0;
  double prev_error_z = 0.0;
  double kp_roll = KP_ROLL;
  double kp_pitch = KP_PITCH;
  double kp_yaw = KP_YAW;
  double kp_z = KP_Z;
  double ki_roll = KI_ROLL;
  double ki_pitch = KI_PITCH;
  double ki_yaw = KI_YAW;
  double ki_z = KI_Z;
  double kd_roll = KD_ROLL;
  double kd_pitch = KD_PITCH;
  double kd_yaw = KD_YAW;
  double kd_z = KD_Z;
  double dt = 0.01;

  // Output Variables
  double output_roll = 0.0;
  double output_pitch = 0.0;
  double output_yaw = 0.0;
  double output_z = 0.0;
  double thrust_fl = 0.0;
  double thrust_fr = 0.0;
  double thrust_rl = 0.0;
  double thrust_rr = 0.0;

  // Get and enable devices and configure drone
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();
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
  WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");

  // Get basic components for tuning
  WbDeviceTag imu_tuning = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu_tuning, timestep);
  WbDeviceTag gps_tuning = wb_robot_get_device("gps");
  wb_gps_enable(gps_tuning, timestep);
  WbDeviceTag motor_tuning =
      wb_robot_get_device("front left propeller"); // Just one motor for test
  wb_motor_set_position(motor_tuning, INFINITY);
  wb_motor_set_velocity(motor_tuning, 0.0);

  // Command line check for tuning
  if (argc > 1) {
    if (strcmp(argv[1], "--tune=twiddle") == 0) {
      tune_pid_twiddle(imu_tuning, gps_tuning, motor_tuning, timestep);
      wb_robot_cleanup();
      return 0;
    } else if (strcmp(argv[1], "--tune=zn") == 0) {
      tune_pid_ziegler_nichols(imu_tuning, gps_tuning, motor_tuning, timestep);
      wb_robot_cleanup();
      return 0;
    }
  }

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor,
                           rear_right_motor};
  int m;
  for (m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  // Wait for 2 seconds
  wb_robot_step(2000);

  // Initial values
  const double *imu_values = wb_inertial_unit_get_roll_pitch_yaw(imu);
  roll = imu_values[0];
  pitch = imu_values[1];
  yaw = imu_values[2];
  const double *gyro_values = wb_gyro_get_values(gyro);
  roll_rate = gyro_values[0];
  pitch_rate = gyro_values[1];
  yaw_rate = gyro_values[2];
  const double *gps_values = wb_gps_get_values(gps);
  pos_x = gps_values[0];
  pos_y = gps_values[1];
  pos_z = gps_values[2];

  // Set initial target values to current position and orientation
  target_roll = roll;
  target_pitch = pitch;
  target_yaw = yaw;
  target_pos_x = pos_x;
  target_pos_y = pos_y;
  target_pos_z = pos_z;

  // Main loop
  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();

    // Blink LEDs and stabilize the camera
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);
    double cam_roll_pos = CLAMP(-0.115 * roll_rate, -0.5, 0.5);
    double cam_pitch_pos = CLAMP(-0.1 * pitch_rate, -0.5, 0.5);
    wb_motor_set_position(camera_roll_motor, cam_roll_pos);
    wb_motor_set_position(camera_pitch_motor, cam_pitch_pos);

    // Get IMU values
    const double *imu_values = wb_inertial_unit_get_roll_pitch_yaw(imu);
    roll = imu_values[0];
    pitch = imu_values[1];
    yaw = imu_values[2];

    // Get gyroscope values
    const double *gyro_values = wb_gyro_get_values(gyro);
    roll_rate = gyro_values[0];
    pitch_rate = gyro_values[1];
    yaw_rate = gyro_values[2];

    // Get GPS values
    const double *gps_values = wb_gps_get_values(gps);
    pos_x = gps_values[0];
    pos_y = gps_values[1];
    pos_z = gps_values[2];

    // Get keyboard input (TEMPORARY)
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
      case WB_KEYBOARD_UP:
        target_pitch -= 0.5;
        break;
      case WB_KEYBOARD_DOWN:
        target_pitch += 0.5;
        break;
      case WB_KEYBOARD_RIGHT:
        target_roll -= 0.5;
        break;
      case WB_KEYBOARD_LEFT:
        target_roll += 0.5;
        break;
      case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
        target_pos_x -= 0.01;
        break;
      case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
        target_pos_x += 0.01;
        break;
      case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
        target_pos_z -= 0.01;
        break;
      case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
        target_pos_z += 0.01;
        break;
      }
      key = wb_keyboard_get_key();
    }

    // PID Control -------------------------------------------------------------
    // -------------------------------------------------------------------------

    // Attitude Control
    error_z = target_pos_z - pos_z;
    integral_z += error_z * dt;
    CLAMP_INTEGRAL(integral_z);
    derivative_z = (error_z - prev_error_z) / dt;
    output_z = kp_z * error_z + ki_z * integral_z + kd_z * derivative_z;
    prev_error_z = error_z;

    // Roll PID
    error_roll = target_roll - roll;
    integral_roll += error_roll * dt;
    CLAMP_INTEGRAL(integral_roll);
    derivative_roll = (error_roll - prev_error_roll) / dt;
    output_roll = kp_roll * error_roll + ki_roll * integral_roll +
                  kd_roll * derivative_roll;
    prev_error_roll = error_roll;

    // Pitch PID
    error_pitch = target_pitch - pitch;
    integral_pitch += error_pitch * dt;
    CLAMP_INTEGRAL(integral_pitch);
    derivative_pitch = (error_pitch - prev_error_pitch) / dt;
    output_pitch = kp_pitch * error_pitch + ki_pitch * integral_pitch +
                   kd_pitch * derivative_pitch;
    prev_error_pitch = error_pitch;

    // Yaw PID
    error_yaw = target_yaw - yaw;
    integral_yaw += error_yaw * dt;
    CLAMP_INTEGRAL(integral_yaw);
    derivative_yaw = (error_yaw - prev_error_yaw) / dt;
    output_yaw =
        kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * derivative_yaw;
    prev_error_yaw = error_yaw;

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------

    // Motor control
    base_thrust = hover_thrust + output_z;
    thrust_fl = CLAMP(base_thrust + output_pitch + output_roll - output_yaw,
                      MIN_THRUST, MAX_THRUST);
    thrust_fr = CLAMP(base_thrust + output_pitch - output_roll + output_yaw,
                      MIN_THRUST, MAX_THRUST);
    thrust_rl = CLAMP(base_thrust - output_pitch + output_roll + output_yaw,
                      MIN_THRUST, MAX_THRUST);
    thrust_rr = CLAMP(base_thrust - output_pitch - output_roll - output_yaw,
                      MIN_THRUST, MAX_THRUST);
    wb_motor_set_velocity(front_left_motor, thrust_fl);
    wb_motor_set_velocity(front_right_motor, -thrust_fr);
    wb_motor_set_velocity(rear_left_motor, -thrust_rl);
    wb_motor_set_velocity(rear_right_motor, thrust_rr);

    // Log Outputs
    printf("[%.2fs] "
           "Roll: T%+6.2f C%+6.2f E%+6.2f O%+6.2f | "
           "Pitch: T%+6.2f C%+6.2f E%+6.2f O%+6.2f | "
           "Yaw: T%+6.2f C%+6.2f E%+6.2f O%+6.2f | "
           "Alt(Z): T%+6.2f C%+6.2f E%+6.2f O%+6.2f | "
           "Motors: FL%+6.2f FR%+6.2f RL%+6.2f RR%+6.2f\n",
           time, target_roll, roll, error_roll, output_roll, target_pitch,
           pitch, error_pitch, output_pitch, target_yaw, yaw, error_yaw,
           output_yaw, target_pos_z, pos_z, error_z, output_z, thrust_fl,
           thrust_fr, thrust_rl, thrust_rr);
  }
}
