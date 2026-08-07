#include <VL53L0X.h>
#include "tof.h"

VL53L0X tof;

void tof_setup() {
    // Start I2C
    Wire.begin(21, 22);

    // Sensor timeout
    tof.setTimeout(500);

    // Initialize sensor
    if (!tof.init())
    {
        Serial.println("Failed to detect VL53L0X");
        while (1);
    }

    // Optional: improve accuracy
    tof.setMeasurementTimingBudget(50000);
    // Start continuous reading
    tof.startContinuous();
}

uint16_t tof_loop() {
    uint16_t distance = tof.readRangeContinuousMillimeters();
    return distance;
}