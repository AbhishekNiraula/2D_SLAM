#pragma once
#include <Arduino.h>

// Servo sweep limits.
#define SERVO_PIN 17
#define SERVO_MIN_DEG 10
#define SERVO_MAX_DEG 110

// Time between 1-degree moves. Increase this if the servo moves too fast.
#define SERVO_STEP_INTERVAL_MS 10

void servo_setup();

// Call repeatedly from loop() to keep the servo sweeping.
void servo_sweep_tick();

// Current physical servo angle, measured from the base_link forward axis.
float servo_get_angle_rad();

// Current commanded servo angle for indexing the scan buffer.
int servo_get_angle_deg();
