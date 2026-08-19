#pragma once
#include <Arduino.h>

// Servo sweep limits.
#define SERVO_PIN 17
#define SERVO_MIN_DEG 5
#define SERVO_MAX_DEG 105

// Time between 1-degree moves. Increase this if the servo moves too fast.
#define SERVO_STEP_INTERVAL_MS 10

void servo_setup();

// Call repeatedly from loop() to keep the servo sweeping.
void servo_sweep_tick();

// Pause scanning while the robot moves. When movement stops, reset to the
// minimum angle and begin a fresh MIN..MAX scan. Returns true on scan start.
bool servo_set_robot_moving(bool moving);

bool servo_is_scanning();

// Current physical servo angle, measured from the base_link forward axis.
float servo_get_angle_rad();

// Current commanded servo angle for indexing the scan buffer.
int servo_get_angle_deg();
