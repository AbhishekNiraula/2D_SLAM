#include <VL53L0X.h>
#include "tof.h"

VL53L0X tof;
static bool tof_ready = false;
static unsigned long next_init_attempt_ms = 0;

static bool try_init_tof()
{
    if (!tof.init())
    {
        tof_ready = false;
        next_init_attempt_ms = millis() + 1000;
        Serial.println("[ToF] VL53L0X init failed; retrying");
        return false;
    }

    // A shorter budget keeps a complete double sweep within the controller's
    // scan phase while still providing stable VL53L0X readings.
    tof.setMeasurementTimingBudget(30000);
    tof.startContinuous(35);
    tof_ready = true;
    Serial.println("[ToF] VL53L0X ready");
    return true;
}

void tof_setup() {
    // Start I2C
    Wire.begin(21, 22);

    // Never allow a stuck I2C transaction to hold the main loop forever.
    // The VL53L0X has its own timeout too; this protects the ESP32-side bus.
    Wire.setTimeOut(20);

    // Sensor timeout
    tof.setTimeout(100);

    // Do not trap the whole firmware in while(1) if the sensor is temporarily
    // unavailable after a reset. tof_loop() will retry initialization.
    try_init_tof();
}

uint16_t tof_loop() {
    if (!tof_ready)
    {
        unsigned long now = millis();
        if ((long)(now - next_init_attempt_ms) >= 0)
            try_init_tof();
        return 0;
    }

    // Give the Wi-Fi/micro-ROS task a scheduling point before entering the
    // sensor wait. This prevents a slow measurement from starving the ESP32
    // while the servo is stepping.
    yield();
    uint16_t distance = tof.readRangeContinuousMillimeters();

    // Return an invalid reading on timeout instead of reusing a partially
    // read value. The caller converts 0 to NaN and will not map that beam.
    if (tof.timeoutOccurred())
    {
        static unsigned long last_timeout_log = 0;
        unsigned long now = millis();
        if (now - last_timeout_log >= 1000)
        {
            last_timeout_log = now;
            Serial.println("[ToF] I2C/measurement timeout");
        }
        tof_ready = false;
        next_init_attempt_ms = now + 250;
        return 0;
    }
    yield();
    return distance;
}
