#include "servo.h"

#include <ESP32Servo.h>

static Servo scan_servo;
static int angle_deg = SERVO_MIN_DEG;
static int direction = 1;
static unsigned long last_step_ms = 0;

void servo_setup()
{
	scan_servo.setPeriodHertz(50);
	scan_servo.attach(SERVO_PIN, 500, 2400);
	scan_servo.write(angle_deg);
}

void servo_sweep_tick()
{
	unsigned long now = millis();
	if (now - last_step_ms < SERVO_STEP_INTERVAL_MS)
		return;

	last_step_ms = now;
	angle_deg += direction;

	if (angle_deg >= SERVO_MAX_DEG)
	{
		angle_deg = SERVO_MAX_DEG;
		direction = -1;
	}
	else if (angle_deg <= SERVO_MIN_DEG)
	{
		angle_deg = SERVO_MIN_DEG;
		direction = 1;
	}

	scan_servo.write(angle_deg);
}

float servo_get_angle_rad()
{
	return angle_deg * (float)M_PI / 180.0f;
}

int servo_get_angle_deg()
{
	return angle_deg;
}
