#pragma once
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <math.h>

// ─────────────────────────────────────────────
//  ROBOT PHYSICAL CONSTANTS  ← edit these
// ─────────────────────────────────────────────
#define WHEEL_RADIUS_M 0.02f  // metres
#define WHEEL_BASE_M 0.11f	  // metres (centre-to-centre, matches actual bot)
#define ENCODER_CPR 358		  // ppr × gear_ratio × quadrature
							  // e.g.= 7 PPR × 4 × 30
#define MAX_WHEEL_RAD_S 10.0f // rad/s — tune to your motor

// ─────────────────────────────────────────────
//  MOTOR DRIVER PINS  (TB6612FNG)
// ─────────────────────────────────────────────
#define PWMA 32
#define AIN1 25
#define AIN2 33
#define PWMB 13
#define BIN1 14
#define BIN2 26
#define STBY 27
#define BUTTON 16

// ─────────────────────────────────────────────
//  ENCODER PINS
// ─────────────────────────────────────────────
#define ENC_RIGHT_C1 23
#define ENC_RIGHT_C2 19
#define ENC_LEFT_C1 18
#define ENC_LEFT_C2 4

// ─────────────────────────────────────────────
//  PWM / TIMING LIMITS
// ─────────────────────────────────────────────
#define MAX_MOTOR_PWM 130
#define MIN_MOTOR_PWM 60
#define CMD_TIMEOUT_MS 500 // watchdog after first cmd_vel

// ─────────────────────────────────────────────
//  GYRO / ENCODER COMPLEMENTARY FILTER
//  Close to 1.0 trusts the gyro almost entirely for short-term
//  rotation (immune to wheel slip); the small remainder lets the
//  encoder-derived heading pull the long-term average back in
//  line, correcting for the gyro's own slow bias drift.
// ─────────────────────────────────────────────
#define GYRO_FILTER_ALPHA 0.98f

// ─────────────────────────────────────────────
//  EXTERN STATE — defined in motor.cpp
// ─────────────────────────────────────────────
extern volatile long enc_right;
extern volatile long enc_left;

extern float odom_x;
extern float odom_y;
extern float odom_theta;
extern float odom_vx;
extern float odom_vtheta;

extern float cmd_linear_x;
extern float cmd_angular_z;
extern unsigned long last_cmd_ms; // ULONG_MAX = "no cmd yet"

extern bool motor_enabled;

// ─────────────────────────────────────────────
//  FUNCTION PROTOTYPES
// ─────────────────────────────────────────────
void IRAM_ATTR isr_right_encoder();
void IRAM_ATTR isr_left_encoder();

void motor_setup();
void motor_stop_all();
void motor_apply_twist(float linear, float angular);

// dt = seconds since last call. Heading is calculated from wheel encoders.
void motor_update_odometry(float dt);

void motor_poll_button();
void motor_drive_tick();

#endif // MOTOR_H
