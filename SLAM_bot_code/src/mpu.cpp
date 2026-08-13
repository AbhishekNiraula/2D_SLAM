#include <Wire.h>
#include "mpu.h"

// ─────────────────────────────────────────────
//  MPU6050 register map (only what we need)
// ─────────────────────────────────────────────
#define MPU_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_GYRO_ZOUT_H 0x47

// ±250 deg/s range (default) → sensitivity 131 LSB per deg/s
#define GYRO_SENS_LSB_PER_DEGS 131.0f

#define BIAS_SAMPLES 200 // ~1 second at 5ms/sample during setup

static float gyro_bias_dps = 0.0f; // measured while stationary at boot

// ── low-level helpers ──────────────────────────
static void mpu_write_reg(uint8_t reg, uint8_t val)
{
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(reg);
	Wire.write(val);
	Wire.endTransmission();
}

static int16_t mpu_read_gyro_z_raw()
{
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(REG_GYRO_ZOUT_H);
	Wire.endTransmission(false);
	Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
	if (Wire.available() < 2)
		return 0;
	int16_t hi = Wire.read();
	int16_t lo = Wire.read();
	return (int16_t)((hi << 8) | lo);
}

// ─────────────────────────────────────────────
//  PUBLIC: mpu_setup — call once in setup(), AFTER Wire.begin()
//  Wakes the sensor, then calibrates gyro-Z bias while the bot
//  MUST be sitting still (called before the bot ever moves).
// ─────────────────────────────────────────────
void mpu_setup()
{
	// wake from sleep (register defaults to sleep=1 on power-up)
	mpu_write_reg(REG_PWR_MGMT_1, 0x00);
	delay(100);

	// explicit ±250 deg/s range (register default, but set anyway)
	mpu_write_reg(REG_GYRO_CONFIG, 0x00);
	delay(10);

	Serial.println("[MPU] Calibrating gyro bias — keep the bot still...");
	long sum = 0;
	for (int i = 0; i < BIAS_SAMPLES; i++)
	{
		sum += mpu_read_gyro_z_raw();
		delay(5);
	}
	float avg_raw = (float)sum / (float)BIAS_SAMPLES;
	gyro_bias_dps = avg_raw / GYRO_SENS_LSB_PER_DEGS;
	Serial.print("[MPU] Gyro-Z bias (deg/s): ");
	Serial.println(gyro_bias_dps, 4);
}

// ─────────────────────────────────────────────
//  PUBLIC: mpu_get_yaw_rate_rad_s — call once per odometry tick
// ─────────────────────────────────────────────
float mpu_get_yaw_rate_rad_s()
{
	int16_t raw = mpu_read_gyro_z_raw();
	float dps = (raw / GYRO_SENS_LSB_PER_DEGS) - gyro_bias_dps;
	return dps * (float)M_PI / 180.0f;
}