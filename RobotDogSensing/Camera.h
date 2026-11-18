#pragma once
#include "SensorTypes.h"
#include <esp_err.h>

struct camera_pins_t {
  int pwdn;
  int reset;
  int xclk;
  int sda;
  int scl;
  int d0;
  int d1;
  int d2;
  int d3;
  int d4;
  int d5;
  int d6;
  int d7;
  int vsync;
  int href;
  int pclk;
};

// Setup only, before camera_init(). pwdn/reset may be -1; all other pins
// must be supplied from the exact board's schematic. Reserves SCCB/I2C1,
// LEDC timer 0/channel 0, and the camera peripheral (I2S0 on classic ESP32).
bool camera_configure(const camera_pins_t *pins);

// Setup can block while the sensor starts. Supported JPEG sizes:
// 160x120, 320x240, 640x480, 800x600, 1024x768, 1280x1024, 1600x1200.
// jpeg_quality: 0..63, lower means higher quality. Without PSRAM only
// 160x120 and 320x240 are accepted, using one framebuffer.
bool camera_init(uint16_t width, uint16_t height, uint8_t jpeg_quality);

// Immediate, cached frame; returns valid=false when nothing fresh is ready
// or another frame is already borrowed. One outstanding lease is allowed.
// Frame data stays unchanged until camera_release_frame(). Do not mutate
// or free it. Copying the struct does not create another ownership lease.
// Finish asynchronous transmission before releasing the frame.
camera_frame_t camera_get_frame(void);
void camera_release_frame(camera_frame_t *frame);

uint32_t camera_captured_frames(void);
esp_err_t camera_last_error(void);
