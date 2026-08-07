// ============================================================
//  SLAM BOT — main.cpp
//  Publishes : /scan  (sensor_msgs/LaserScan)
//              /odom  (nav_msgs/Odometry)
//              /tf    (geometry_msgs/TransformStamped)
//  Subscribes: /cmd_vel (geometry_msgs/Twist)
// ============================================================
// Command to run
// rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
// ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10 // go straight
// ros2 run teleop_twist_keyboard teleop_twist_keyboard

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/laser_scan.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include "tof.h"
#include "motor.h"

// ─────────────────────────────────────────────
//  WiFi & Agent
// ─────────────────────────────────────────────
char ssid[] = "umastha15_dhrn";
char psw[] = "TNnBUSMQWQ@URHM";

IPAddress agent_ip(192, 168, 1, 69);
size_t agent_port = 8888;

// ─────────────────────────────────────────────
//  Publish rates
// ─────────────────────────────────────────────
const long SCAN_INTERVAL_MS = 100;
const long ODOM_INTERVAL_MS = 20;
const float ODOM_DT = ODOM_INTERVAL_MS / 1000.0f;

// ─────────────────────────────────────────────
//  micro-ROS handles
// ─────────────────────────────────────────────
rcl_publisher_t scan_pub;
rcl_publisher_t odom_pub;
rcl_publisher_t tf_pub;
rcl_subscription_t cmd_vel_sub;

sensor_msgs__msg__LaserScan scan_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__TransformStamped tf_transform;
geometry_msgs__msg__Twist cmd_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

float ranges[1];
float intensities[1];

// ─────────────────────────────────────────────
//  State machine
// ─────────────────────────────────────────────
enum AgentState
{
	WAITING_AGENT,
	AGENT_AVAILABLE,
	AGENT_CONNECTED,
	AGENT_DISCONNECTED
} state;

unsigned long last_ping_ms = 0;
unsigned long last_scan_ms = 0;
unsigned long last_odom_ms = 0;
const long PING_INTERVAL = 500;

// ─────────────────────────────────────────────
//  Forward declarations
// ─────────────────────────────────────────────
bool create_entities();
void destroy_entities();
void wifi_setup();
void init_scan_msg();
void init_odom_msg();
void cmd_vel_callback(const void *msgin);
void publish_scan(uint16_t distance_mm, int64_t time_ns);
void publish_odom(int64_t time_ns);

// ─────────────────────────────────────────────
//  WiFi setup
// ─────────────────────────────────────────────
void wifi_setup()
{
	WiFi.setAutoReconnect(true);
	WiFi.persistent(true);
	WiFi.begin(ssid, psw);

	Serial.print("[WiFi] Connecting");
	int attempts = 0;
	while (WiFi.status() != WL_CONNECTED && attempts < 30)
	{
		digitalWrite(2, HIGH);
		delay(250);
		digitalWrite(2, LOW);
		delay(250);
		Serial.print(".");
		attempts++;
	}
	if (WiFi.status() == WL_CONNECTED)
	{
		Serial.println(" Connected!");
		Serial.print("[WiFi] IP: ");
		Serial.println(WiFi.localIP());
		digitalWrite(2, HIGH);
	}
	else
	{
		Serial.println(" FAILED!");
		digitalWrite(2, LOW);
	}
}

// ─────────────────────────────────────────────
//  Message initialisers
// ─────────────────────────────────────────────
void init_scan_msg()
{
	static char frame_id[] = "laser";
	scan_msg.header.frame_id.data = frame_id;
	scan_msg.header.frame_id.size = strlen(frame_id);
	scan_msg.header.frame_id.capacity = strlen(frame_id) + 1;

	scan_msg.angle_min = -0.0175f;		// -1 degree
	scan_msg.angle_max = 0.0175f;		// +1 degree
	scan_msg.angle_increment = 0.0349f; // 2 degrees → gives exactly 1 ray
	scan_msg.time_increment = 0.0f;
	scan_msg.scan_time = 0.1f;
	scan_msg.range_min = 0.05f;
	scan_msg.range_max = 2.0f;

	ranges[0] = 0.0f;
	ranges[1] = 0.0f; // ← second ray, same distance
	intensities[0] = 50.0f;
	intensities[1] = 50.0f;
	scan_msg.ranges.data = ranges;
	scan_msg.ranges.size = 2; // ← 2 rays
	scan_msg.ranges.capacity = 2;
	scan_msg.intensities.data = intensities;
	scan_msg.intensities.size = 2;
	scan_msg.intensities.capacity = 2;
}

void init_odom_msg()
{
	static char odom_frame[] = "odom";
	static char base_frame[] = "base_link";

	odom_msg.header.frame_id.data = odom_frame;
	odom_msg.header.frame_id.size = strlen(odom_frame);
	odom_msg.header.frame_id.capacity = strlen(odom_frame) + 1;
	odom_msg.child_frame_id.data = base_frame;
	odom_msg.child_frame_id.size = strlen(base_frame);
	odom_msg.child_frame_id.capacity = strlen(base_frame) + 1;

	odom_msg.pose.covariance[0] = 0.001;
	odom_msg.pose.covariance[7] = 0.001;
	odom_msg.pose.covariance[35] = 0.01;
	odom_msg.twist.covariance[0] = 0.001;
	odom_msg.twist.covariance[35] = 0.01;

	tf_transform.header.frame_id.data = odom_frame;
	tf_transform.header.frame_id.size = strlen(odom_frame);
	tf_transform.header.frame_id.capacity = strlen(odom_frame) + 1;
	tf_transform.child_frame_id.data = base_frame;
	tf_transform.child_frame_id.size = strlen(base_frame);
	tf_transform.child_frame_id.capacity = strlen(base_frame) + 1;
}

// ─────────────────────────────────────────────
//  cmd_vel callback
// ─────────────────────────────────────────────
void cmd_vel_callback(const void *msgin)
{
	const geometry_msgs__msg__Twist *msg =
		(const geometry_msgs__msg__Twist *)msgin;
	cmd_linear_x = msg->linear.x;
	cmd_angular_z = msg->angular.z;
	last_cmd_ms = millis(); // re-arms the watchdog
}

// ─────────────────────────────────────────────
//  Publish helpers
// ─────────────────────────────────────────────
void publish_scan(uint16_t distance_mm, int64_t time_ns)
{
	scan_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
	scan_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);
	scan_msg.ranges.size = 2;
	scan_msg.intensities.size = 2;
	float d = (distance_mm == 0 || distance_mm >= 8190)
				  ? scan_msg.range_max
				  : distance_mm / 1000.0f;

	ranges[0] = d;
	ranges[1] = d; // ← both rays same distance

	rcl_publish(&scan_pub, &scan_msg, NULL);
}

void publish_odom(int64_t time_ns)
{
	motor_update_odometry(ODOM_DT);

	odom_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
	odom_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);

	odom_msg.pose.pose.position.x = odom_x;
	odom_msg.pose.pose.position.y = odom_y;
	odom_msg.pose.pose.position.z = 0.0;

	odom_msg.pose.pose.orientation.x = 0.0;
	odom_msg.pose.pose.orientation.y = 0.0;
	odom_msg.pose.pose.orientation.z = sinf(odom_theta / 2.0f);
	odom_msg.pose.pose.orientation.w = cosf(odom_theta / 2.0f);

	odom_msg.twist.twist.linear.x = odom_vx;
	odom_msg.twist.twist.angular.z = odom_vtheta;

	rcl_publish(&odom_pub, &odom_msg, NULL);

	tf_transform.header.stamp = odom_msg.header.stamp;
	tf_transform.transform.translation.x = odom_x;
	tf_transform.transform.translation.y = odom_y;
	tf_transform.transform.translation.z = 0.0;
	tf_transform.transform.rotation = odom_msg.pose.pose.orientation;

	rcl_publish(&tf_pub, &tf_transform, NULL);
}

// ─────────────────────────────────────────────
//  Create / destroy micro-ROS entities
// ─────────────────────────────────────────────
bool create_entities()
{
	rmw_uros_sync_session(1000);

	if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
	{
		Serial.println("[ERR] support init");
		return false;
	}
	if (rclc_node_init_default(&node, "slam_bot_node", "", &support) != RCL_RET_OK)
	{
		Serial.println("[ERR] node init");
		return false;
	}
	if (rclc_publisher_init_default(
			&scan_pub, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
			"scan") != RCL_RET_OK)
	{
		Serial.println("[ERR] scan pub");
		return false;
	}
	if (rclc_publisher_init_default(
			&odom_pub, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
			"/odom") != RCL_RET_OK)
	{
		Serial.println("[ERR] odom pub");
		return false;
	}
	if (rclc_publisher_init_default(
			&tf_pub, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped),
			"/tf_raw") != RCL_RET_OK)
	{
		Serial.println("[ERR] tf pub");
		return false;
	}
	if (rclc_subscription_init_default(
			&cmd_vel_sub, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
			"/cmd_vel") != RCL_RET_OK)
	{
		Serial.println("[ERR] cmd_vel sub");
		return false;
	}
	if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
	{
		Serial.println("[ERR] executor init");
		return false;
	}
	if (rclc_executor_add_subscription(
			&executor, &cmd_vel_sub, &cmd_msg,
			&cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK)
	{
		Serial.println("[ERR] add subscription");
		return false;
	}
	return true;
}

void destroy_entities()
{
	rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
	(void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

	rcl_publisher_fini(&scan_pub, &node);
	rcl_publisher_fini(&odom_pub, &node);
	rcl_publisher_fini(&tf_pub, &node);
	rcl_subscription_fini(&cmd_vel_sub, &node);
	rcl_node_fini(&node);
	rclc_executor_fini(&executor);
	rclc_support_fini(&support);

	// Reset cmd watchdog — no agent means no cmd_vel is coming
	last_cmd_ms = ULONG_MAX;
}

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup()
{
	// very important for micro ros esp32 stability — prevents brownout resets when WiFi is used
	// BROWN OUT DETECT is enabled by default on ESP32, which causes a reset when the voltage drops (e.g. when driving motors). Disable it to prevent unexpected resets.
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

	Serial.begin(115200);
	pinMode(2, OUTPUT);
	digitalWrite(2, LOW);
	delay(2000);

	tof_setup();
	motor_setup();

	wifi_setup();
	set_microros_wifi_transports(ssid, psw, agent_ip, agent_port);
	delay(2000);

	allocator = rcl_get_default_allocator();
	init_scan_msg();
	init_odom_msg();

	state = WAITING_AGENT;
	// Serial.println("[Setup] Ready — waiting for micro-ROS agent...");
}

// ─────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────
void loop()
{
	unsigned long now = millis();

	// Button is always active regardless of agent state
	motor_poll_button();

	// WiFi watchdog
	if (WiFi.status() != WL_CONNECTED)
	{
		Serial.println("[WiFi] Lost — reconnecting...");
		digitalWrite(2, LOW);
		wifi_setup();
		set_microros_wifi_transports(ssid, psw, agent_ip, agent_port);
		delay(500);
		state = WAITING_AGENT;
		return;
	}

	uint16_t distance_mm = tof_loop();

	switch (state)
	{

	case WAITING_AGENT:
		if (now - last_ping_ms >= PING_INTERVAL)
		{
			last_ping_ms = now;
			if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
			{
				state = AGENT_AVAILABLE;
				Serial.println("[Agent] Found!");
			}
			else
			{
				Serial.println("[Agent] Waiting...");
			}
		}
		break;

	case AGENT_AVAILABLE:
		if (create_entities())
		{
			state = AGENT_CONNECTED;
			Serial.println("[micro-ROS] Publishing /scan  /odom  /tf");
		}
		else
		{
			Serial.println("[micro-ROS] Entity creation failed — retrying...");
			destroy_entities();
			state = WAITING_AGENT;
		}
		break;

	case AGENT_CONNECTED:
	{
		if (rmw_uros_epoch_nanos() < 1000000000000LL)
		{
			rmw_uros_sync_session(10);
		}
		int64_t time_ns = rmw_uros_epoch_nanos();

		if (now - last_scan_ms >= SCAN_INTERVAL_MS)
		{
			last_scan_ms = now;
			publish_scan(distance_mm, time_ns);
			Serial.printf("[Scan] %.3f m  (%u mm)\n", ranges[0], distance_mm);
		}

		if (now - last_odom_ms >= ODOM_INTERVAL_MS)
		{
			last_odom_ms = now;
			publish_odom(time_ns);
			Serial.printf("[Odom] x=%.3f  y=%.3f  th=%.3f\n",
						  odom_x, odom_y, odom_theta);
		}

		// Spin executor first so cmd_vel is fresh before driving
		if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)) != RCL_RET_OK)
		{
			Serial.println("[micro-ROS] Agent disconnected!");
			state = AGENT_DISCONNECTED;
			break;
		}

		motor_drive_tick();
		break;
	}

	case AGENT_DISCONNECTED:
		motor_stop_all();
		destroy_entities(); // also resets last_cmd_ms → ULONG_MAX
		state = WAITING_AGENT;
		Serial.println("[micro-ROS] Cleaned up — waiting for agent...");
		break;
	}

	delay(5);
}