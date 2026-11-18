#pragma once
#include <Arduino.h>

// Project PDF interfaces. Units and the original field order are preserved.
struct imu_reading_t {
  float accel_g[3];
  float gyro_dps[3];
  uint32_t t_ms;
  bool valid;
};
struct tof_reading_t {
  uint16_t distance_mm;
  uint32_t t_ms;
  bool valid;
};
enum tof_sensor_t { TOF_FRONT, TOF_DOWN };
struct sensing_pins_t {
  int sda;
  int scl;
  int front_xshut;
  int down_xshut;
};
struct init_status_t {
  bool i2c;
  bool imu;
  bool front_tof;
  bool down_tof;
};
struct audio_chunk_t {
  int16_t *samples;
  size_t n_samples;
  uint32_t t_ms;  // First sample, in MCU millis() time.
  bool valid;
  uint32_t lease; // Added: ownership token; treat as opaque.
};
struct camera_frame_t {
  const uint8_t *data;
  size_t len;
  uint16_t width;
  uint16_t height;
  uint32_t t_ms;
  bool valid;
  uint32_t lease; // Added: ownership token; treat as opaque.
};
