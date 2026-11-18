#pragma once
#include "SensorTypes.h"
#include "Microphone.h"
#include "Camera.h"

// 1: IMU + both ToFs; 2: microphone; 3: camera; 4: all (requires a verified pin map).
// Override with -DDRIVER_TEST_MODE=N when building with arduino-cli.
#ifndef DRIVER_TEST_MODE
#define DRIVER_TEST_MODE 1
#endif
#if DRIVER_TEST_MODE < 1 || DRIVER_TEST_MODE > 4
#error "DRIVER_TEST_MODE must be 1, 2, 3, or 4"
#endif

constexpr bool TEST_SENSING = DRIVER_TEST_MODE == 1 || DRIVER_TEST_MODE == 4;
constexpr bool TEST_MIC = DRIVER_TEST_MODE == 2 || DRIVER_TEST_MODE == 4;
constexpr bool TEST_CAMERA = DRIVER_TEST_MODE == 3 || DRIVER_TEST_MODE == 4;
constexpr uint32_t TEST_SERIAL_BAUD = 921600;
constexpr uint32_t MIC_SAMPLE_RATE_HZ = 16000;
constexpr uint16_t CAMERA_WIDTH = 320;
constexpr uint16_t CAMERA_HEIGHT = 240;
constexpr uint8_t CAMERA_JPEG_QUALITY = 15; // 0..63; smaller numbers mean higher quality.

#if !CONFIG_IDF_TARGET_ESP32S3 && !CONFIG_IDF_TARGET_ESP32
#error "This project targets the original ESP32 or ESP32-S3, using Arduino ESP32 core 3.x."
#endif

// =====================================================================
// EDIT HERE: ESP32-S3 TEST WIRING. CHANGE TO MATCH YOUR ACTUAL CONNECTIONS.
// Kept unchanged when compiling for another board: these are NOT ESP32-CAM
// integration pins. In particular GPIO6..11 are flash pins on classic ESP32.
// The final integration owner must replace them for the final hardware.
// =====================================================================
constexpr sensing_pins_t SENSING_PINS = {8, 9, 10, 11};
constexpr mic_pins_t MIC_PINS = {4, 5, 6, false}; // BCLK, WS, DIN, right channel?

// CAMERA DEFAULT ONLY: AI-Thinker ESP32-CAM + OV2640. Not an S3 pin map.
// pwdn, reset, xclk, sda, scl, d0..d7, vsync, href, pclk.
// Camera testing is optional and is outside the current bench test.
constexpr camera_pins_t CAMERA_PINS = {
    32, -1, 0, 26, 27, 5, 18, 19, 21, 36, 39, 34, 35, 25, 23, 22};
// After supplying a VERIFIED S3 camera map, change this to true for S3 testing.
#if CONFIG_IDF_TARGET_ESP32
constexpr bool CAMERA_PIN_MAP_CONFIRMED = true;
#else
constexpr bool CAMERA_PIN_MAP_CONFIRMED = false;
#endif
