#include "motor.h"

// ============================================================
//  motor.cpp — all motor, encoder, and odometry definitions
// ============================================================

// ─────────────────────────────────────────────
//  ENCODER COUNTERS  (volatile — ISR-touched)
// ─────────────────────────────────────────────
volatile long enc_right = 0;
volatile long enc_left = 0;

static long last_enc_right = 0;
static long last_enc_left = 0;

// ─────────────────────────────────────────────
//  ODOMETRY STATE
// ─────────────────────────────────────────────
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;
float odom_vx = 0.0f;
float odom_vtheta = 0.0f;

// ─────────────────────────────────────────────
//  CMD_VEL STATE
// ─────────────────────────────────────────────
float cmd_linear_x = 0.0f;
float cmd_angular_z = 0.0f;

unsigned long last_cmd_ms = ULONG_MAX;

// ─────────────────────────────────────────────
//  BUTTON STATE
// ─────────────────────────────────────────────
bool motor_enabled = false;
bool last_btn_state = HIGH;

// ─────────────────────────────────────────────
//  ENCODER ISRs
// ─────────────────────────────────────────────
void IRAM_ATTR isr_right_encoder()
{
	if (digitalRead(ENC_RIGHT_C2) == HIGH)
		enc_right++;
	else
		enc_right--;
}

void IRAM_ATTR isr_left_encoder()
{
	if (digitalRead(ENC_LEFT_C2) == HIGH)
		enc_left++;
	else
		enc_left--;
}

// ─────────────────────────────────────────────
//  MOTOR PRIMITIVES  (file-local)
// ─────────────────────────────────────────────
static void _setMotorA(int speed)
{
	if (speed > 0)
	{
		digitalWrite(AIN1, HIGH);
		digitalWrite(AIN2, LOW);
		ledcWrite(0, (uint32_t)constrain(speed, MIN_MOTOR_PWM, MAX_MOTOR_PWM));
	}
	else if (speed < 0)
	{
		digitalWrite(AIN1, LOW);
		digitalWrite(AIN2, HIGH);
		ledcWrite(0, (uint32_t)constrain(-speed, MIN_MOTOR_PWM, MAX_MOTOR_PWM));
	}
	else
	{
		ledcWrite(0, 0);
	}
}

static void _setMotorB(int speed)
{
	if (speed > 0)
	{
		digitalWrite(BIN1, HIGH);
		digitalWrite(BIN2, LOW);
		ledcWrite(1, (uint32_t)constrain(speed, MIN_MOTOR_PWM, MAX_MOTOR_PWM));
	}
	else if (speed < 0)
	{
		digitalWrite(BIN1, LOW);
		digitalWrite(BIN2, HIGH);
		ledcWrite(1, (uint32_t)constrain(-speed, MIN_MOTOR_PWM, MAX_MOTOR_PWM));
	}
	else
	{
		ledcWrite(1, 0);
	}
}

// ─────────────────────────────────────────────
//  PUBLIC: stop
// ─────────────────────────────────────────────
void motor_stop_all()
{
	ledcWrite(0, 0);
	ledcWrite(1, 0);
}

// ─────────────────────────────────────────────
//  PUBLIC: differential drive  cmd_vel → PWM
// ─────────────────────────────────────────────
void motor_apply_twist(float linear, float angular)
{
	float v_right = (linear + angular * (WHEEL_BASE_M / 2.0f)) / WHEEL_RADIUS_M;
	float v_left = (linear - angular * (WHEEL_BASE_M / 2.0f)) / WHEEL_RADIUS_M;

	int pwm_r = (int)(v_right / MAX_WHEEL_RAD_S * MAX_MOTOR_PWM);
	int pwm_l = (int)(v_left / MAX_WHEEL_RAD_S * MAX_MOTOR_PWM);

	pwm_r = constrain(pwm_r, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
	pwm_l = constrain(pwm_l, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

	_setMotorA(pwm_l); // Motor A = LEFT
	_setMotorB(pwm_r); // Motor B = RIGHT
}

// ─────────────────────────────────────────────
//  PUBLIC: dead-reckoning odometry update, fused with gyro-Z
//  Call at a fixed rate; dt = interval in seconds.
//  gyro_yaw_rate_rad_s = live reading from mpu_get_yaw_rate_rad_s()
// ─────────────────────────────────────────────
void motor_update_odometry(float dt, float gyro_yaw_rate_rad_s)
{
	noInterrupts();
	long cur_right = enc_right;
	long cur_left = enc_left;
	interrupts();

	long d_right = cur_right - last_enc_right;
	long d_left = last_enc_left - cur_left; // left encoder is reversed
	last_enc_right = cur_right;
	last_enc_left = cur_left;

	const float M_PER_TICK = (2.0f * (float)M_PI * WHEEL_RADIUS_M) / (float)ENCODER_CPR;
	float dist_r = d_right * M_PER_TICK;
	float dist_l = d_left * M_PER_TICK;

	float dist_c = (dist_r + dist_l) / 2.0f;

	// ── heading: complementary-filter encoder vs. gyro ──────────
	float delta_th_enc = (dist_r - dist_l) / WHEEL_BASE_M;
	float delta_th_gyro = gyro_yaw_rate_rad_s * dt;
	float delta_th = GYRO_FILTER_ALPHA * delta_th_gyro + (1.0f - GYRO_FILTER_ALPHA) * delta_th_enc;

	odom_x += dist_c * cosf(odom_theta + delta_th / 2.0f);
	odom_y += dist_c * sinf(odom_theta + delta_th / 2.0f);
	odom_theta += delta_th;

	while (odom_theta > (float)M_PI)
		odom_theta -= 2.0f * (float)M_PI;
	while (odom_theta < -(float)M_PI)
		odom_theta += 2.0f * (float)M_PI;

	odom_vx = dist_c / dt;
	odom_vtheta = delta_th / dt;
}

// ─────────────────────────────────────────────
//  PUBLIC: motor_setup — call once in setup()
// ─────────────────────────────────────────────
void motor_setup()
{
	pinMode(AIN1, OUTPUT);
	pinMode(AIN2, OUTPUT);
	pinMode(BIN1, OUTPUT);
	pinMode(BIN2, OUTPUT);
	pinMode(STBY, OUTPUT);
	digitalWrite(STBY, HIGH);

	pinMode(ENC_RIGHT_C1, INPUT_PULLUP);
	pinMode(ENC_RIGHT_C2, INPUT_PULLUP);
	pinMode(ENC_LEFT_C1, INPUT_PULLUP);
	pinMode(ENC_LEFT_C2, INPUT_PULLUP);

	pinMode(BUTTON, INPUT_PULLUP);

	ledcSetup(0, 1000, 8);
	ledcAttachPin(PWMA, 0);
	ledcSetup(1, 1000, 8);
	ledcAttachPin(PWMB, 1);

	attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_C1), isr_right_encoder, RISING);
	attachInterrupt(digitalPinToInterrupt(ENC_LEFT_C1), isr_left_encoder, RISING);
}

// ─────────────────────────────────────────────
//  PUBLIC: motor_poll_button — call every loop()
// ─────────────────────────────────────────────
void motor_poll_button()
{
	bool cur = digitalRead(BUTTON);
	if (last_btn_state == HIGH && cur == LOW)
	{
		motor_enabled = !motor_enabled;
		if (!motor_enabled)
		{
			motor_stop_all();
			Serial.println("[Motor] Disabled by button");
		}
		else
		{
			last_cmd_ms = millis();
			Serial.println("[Motor] Enabled by button");
		}
		delay(200); // debounce
	}
	last_btn_state = cur;
}

// ─────────────────────────────────────────────
//  PUBLIC: motor_drive_tick — call in AGENT_CONNECTED
// ─────────────────────────────────────────────
void motor_drive_tick()
{
	if (!motor_enabled)
	{
		motor_stop_all();
		return;
	}

	if (last_cmd_ms == ULONG_MAX)
	{
		motor_stop_all();
		return;
	}

	if ((millis() - last_cmd_ms) > CMD_TIMEOUT_MS)
	{
		motor_stop_all();
	}
	else
	{
		motor_apply_twist(cmd_linear_x, cmd_angular_z);
	}
}