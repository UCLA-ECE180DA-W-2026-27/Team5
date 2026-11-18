#pragma once
#include "SensorTypes.h"

// Call once from setup(). Startup performs device reset/calibration and can wait.
// Requires MPU6050 + VL53L0X (not VL53L1X), and Adafruit VL53L0X 1.2.5.
// This module owns Wire after init: coordinate any other I2C users explicitly.
// Repeating init with the same pins returns the original initialization status;
// changing pins or retrying failed devices requires restarting the board.
init_status_t sensing_init(const sensing_pins_t *pins);
init_status_t sensing_status(void);

// Optional worker wake-up; sampling also runs automatically in the background.
// These functions do not perform I2C operations or wait for a new measurement.
void sensing_update(void);
imu_reading_t imu_read(void);
tof_reading_t tof_read(tof_sensor_t sensor);

// Optional measured static gyro bias, subtracted from subsequent readings.
// Determine it while the assembled robot is stationary; defaults are zero.
void imu_set_gyro_bias(float x_dps, float y_dps, float z_dps);
