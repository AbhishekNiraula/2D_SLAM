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

// Bot1 Niraj Old
// // ─────────────────────────────────────────────
// //  MOTOR DRIVER PINS  (TB6612FNG)
// // ─────────────────────────────────────────────
// #define PWMA 32
// #define AIN1 25
// #define AIN2 33
// #define PWMB 13
// // ============================================================
// //  motor.h — declarations
// // ============================================================
// #define BIN1 14
// #define BIN2 26
// #define STBY 27
// #define BUTTON 16

// // ─────────────────────────────────────────────
// //  ENCODER PINS
// // ─────────────────────────────────────────────
// #define ENC_RIGHT_C1 23
// #define ENC_RIGHT_C2 19
// #define ENC_LEFT_C1 18
// #define ENC_LEFT_C2 4

// Bot2 New Minor Project
// ─────────────────────────────────────────────
//  MOTOR DRIVER PINS  (TB6612FNG)
// ─────────────────────────────────────────────
#define PWMA 32
#define AIN1 25
#define AIN2 33
#define PWMB 13
// ============================================================
//  motor.h — declarations
// ============================================================
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

// ── BUG FIX 1 ──────────────────────────────────────────────
//  CMD_TIMEOUT_MS was 500 ms.  Because last_cmd_ms starts at 0,
//  millis()-0 is already > 500 on the very first call, so
//  motor_drive_tick() immediately stops the motors even when
//  the button is pressed.
//
//  Fix: use ULONG_MAX as the "never received a cmd_vel" sentinel.
//  When the button is pressed without an agent the motors run at
//  a fixed forward speed (see motor_drive_tick).  Once cmd_vel
//  arrives the watchdog takes over normally.
// ───────────────────────────────────────────────────────────
#define CMD_TIMEOUT_MS 500 // watchdog after first cmd_vel

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

// BUG FIX 2 — motor_enabled must be extern so main.cpp's
// motor_poll_button() and motor_drive_tick() share the same copy.
extern bool motor_enabled;

// ─────────────────────────────────────────────
//  FUNCTION PROTOTYPES
// ─────────────────────────────────────────────
void IRAM_ATTR isr_right_encoder();
void IRAM_ATTR isr_left_encoder();

void motor_setup();
void motor_stop_all();
void motor_apply_twist(float linear, float angular);
void motor_update_odometry(float dt);
void motor_poll_button();
void motor_drive_tick();

#endif // MOTOR_H