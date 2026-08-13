#pragma once
#ifndef MPU_H
#define MPU_H

#include <Arduino.h>

// ─────────────────────────────────────────────
//  PUBLIC API
// ─────────────────────────────────────────────
void mpu_setup();
float mpu_get_yaw_rate_rad_s(); // call once per odometry update; returns rad/s

#endif // MPU_H