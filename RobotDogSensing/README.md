# RobotDogSensing

Sensor and media drivers for the ECE 180D robotic dog project. This package supplies measurements to the local behavior logic and audio/image buffers to the wireless subsystem. Behavior decisions, motor control, networking, speech recognition, and battery monitoring belong to other modules.

## Hardware and dependencies

- MPU6050 accelerometer/gyroscope.
- Two VL53L0X distance sensors: front-facing and downward-facing.
- Two INMP441 I2S microphones, configured as left and right channels.
- OV2640 JPEG camera; the default camera pin map targets AI-Thinker ESP32-CAM.
- Arduino ESP32 core 3.3.12 (3.x / ESP-IDF 5 APIs).
- Standard Adafruit VL53L0X library 1.2.5. The IMU uses direct register access; the camera and I2S APIs are included with the ESP32 core.

The sensor and microphone pin defaults target an ESP32-S3 bench setup. They remain unchanged when selecting another compilation target and are **not an ESP32-CAM integration pin map**. Camera defaults target a different board. Final hardware integration requires a board-specific configuration.

## Files

| File | Responsibility |
| --- | --- |
| `SensorTypes.h` | Shared measurement/media types, validity flags, timestamps, and buffer ownership tokens. |
| `Sensing.h` / `Sensing.cpp` | MPU6050 setup and readings; independent startup, address assignment, and distance readings for two VL53L0X sensors. |
| `Microphone.h` / `Microphone.cpp` | Continuous stereo I2S acquisition with selectable left, right, or averaged mono PCM output. |
| `Camera.h` / `Camera.cpp` | Background JPEG acquisition, cached frame access, and framebuffer ownership. |
| `DriverConfig.h` | Bench pin assignments, media settings, and test-mode selection. |
| `RobotDogSensing.ino` | Bench-test application that initializes selected drivers and reports measurements over Serial. It is not the robot behavior state machine. |
| `capture_test.py` | Optional computer-side utility for saving serial PCM transfers as WAV files or receiving JPEG images. |
| `.gitignore` | Excludes build products, generated captures, and local settings. |

All source files are in one Arduino sketch directory, `RobotDogSensing`.

## Data flow and module boundaries

```text
MPU6050 + front/down VL53L0X -> Sensing -> local behavior logic
INMP441 pair               -> Microphone -> wireless audio consumer -> SBC
OV2640                     -> Camera -> wireless image consumer -> SBC
```

Sensing reports measurements and their validity. Edge/lift/flip classification and the resulting motion or display responses are not implemented here. Audio is captured continuously; this package does not run wake-word detection. Name and command recognition are intended to run on the SBC.

## Public interfaces

### IMU and distance sensing

```cpp
init_status_t sensing_init(const sensing_pins_t *pins);
init_status_t sensing_status(void);
void sensing_update(void);
imu_reading_t imu_read(void);
tof_reading_t tof_read(tof_sensor_t sensor);
void imu_set_gyro_bias(float x_dps, float y_dps, float z_dps);
```

- `TOF_FRONT` and `TOF_DOWN` identify the two distance sensors.
- `accel_g[3]` is acceleration in g; `gyro_dps[3]` is angular velocity in degrees per second. Axes follow the physical MPU6050, not an automatically established robot coordinate frame.
- `distance_mm` is distance in millimeters. An invalid reading is not evidence of safe clearance.
- `t_ms` is the measurement-read time in MCU milliseconds. `valid` is false after a failed measurement or when the cached reading becomes stale.
- Target acquisition rates are 100 Hz for the IMU and 20 Hz per ToF. Freshness limits are 100 ms and 200 ms respectively; actual timing depends on hardware and load.
- `sensing_status()` reports initialization results, not continuous device-presence status. Runtime consumers use each reading's `valid` flag.
- Both ToFs are reset through independent XSHUT pins, then assigned addresses 0x30 (front) and 0x31 (down). The MPU6050 is detected at 0x68 or 0x69.
- Gyroscope bias defaults to zero. The optional bias setter applies externally determined stationary offsets.

### Microphone

```cpp
bool mic_configure(const mic_pins_t *pins);
bool mic_init(uint32_t sample_rate_hz);
bool mic_select_channel(mic_channel_t channel);
audio_chunk_t mic_read(size_t max_samples);
void mic_release(audio_chunk_t *chunk);
uint32_t mic_sample_rate_hz();
uint32_t mic_dropped_chunks();
```

The INMP441 pair shares BCLK, WS, and the serial data input. Opposite L/R levels select the two slots. Both channels are captured internally; `MIC_LEFT`, `MIC_RIGHT`, or `MIC_MIX` selects the mono output. Mixing is an arithmetic average, not sound localization.

The output is signed 16-bit mono PCM, 16 kHz by default. Each immediate read returns at most `min(max_samples, 320)` samples. No available data or no free borrow slot produces `valid=false`. Pending buffers are bounded; overflow drops old pending audio and increments the drop counter without overwriting borrowed data.

`t_ms` approximates the first sample time using MCU time and sample counts. It is not a precise hardware timestamp for microphone direction-of-arrival calculations.

### Camera

```cpp
bool camera_configure(const camera_pins_t *pins);
bool camera_init(uint16_t width, uint16_t height, uint8_t jpeg_quality);
camera_frame_t camera_get_frame(void);
void camera_release_frame(camera_frame_t *frame);
uint32_t camera_captured_frames(void);
esp_err_t camera_last_error(void);
```

Frames contain JPEG bytes, length, dimensions, capture timestamp, validity, and an ownership token. `camera_get_frame()` immediately returns a cached fresh frame or `valid=false`. Frames older than one second are rejected. Capture timestamps come from the first DMA buffer of the frame.

The camera uses at most two native framebuffers with PSRAM, or one without PSRAM, and does not copy a third complete frame. Without PSRAM, accepted resolutions are limited to 160x120 and 320x240. JPEG quality ranges from 0 to 63; lower values mean higher quality.

## Buffer ownership

A valid audio chunk remains unchanged until `mic_release()`; up to four chunks can be outstanding. A valid camera frame remains unchanged until `camera_release_frame()`; one frame can be outstanding.

An asynchronous wireless consumer must finish using the borrowed memory before releasing it, or first copy the payload into its own buffer. Copying a C++ structure does not create another ownership lease. Release functions clear the supplied structure and use the opaque `lease` token to reject duplicate or stale releases.

The `lease` fields and `mic_release()` extend the planning document's interfaces to make buffer lifetime explicit. Pointers, ownership tokens, and raw C++ structure layouts are not a network packet format. The wireless module serializes the payload and required metadata explicitly.

## Runtime and shared resources

Initialization is setup-only and may wait for hardware reset or calibration. Background tasks perform acquisition. Measurement getters return cached data without waiting for I2C, DMA, a new frame, or a complete utterance. `sensing_update()` is an optional worker notification; acquisition also runs automatically.

Configuration is startup-only. Repeating sensing initialization with the same pins returns the existing initialization result; changing pins or retrying failed devices requires a restart. The microphone accepts repeated initialization at the same rate, while the camera rejects repeated initialization after startup.

Resource assignments in the supported core configuration:

- Sensing owns Arduino `Wire` / I2C0. Other users of this bus require coordinated access.
- Camera SCCB uses I2C1. The driver rejects a core configuration that does not reserve that controller for the camera.
- Microphone uses I2S1; the original ESP32 camera uses I2S0.
- Camera reserves LEDC timer 0 / channel 0.

## Bench configuration

| Setting | Default |
| --- | --- |
| S3 sensing SDA / SCL | GPIO8 / GPIO9 |
| Front / down XSHUT | GPIO10 / GPIO11 |
| S3 microphone BCLK / WS / DIN | GPIO4 / GPIO5 / GPIO6 |
| Serial baud | 921600 |
| Camera image | 320x240 JPEG, quality 15 |

`DRIVER_TEST_MODE` selects sensing (1), microphone (2), camera (3), or all modules (4). These modes describe the bench application, not alternative driver APIs. The current S3 configuration disables camera startup because its default camera map belongs to AI-Thinker ESP32-CAM. The sketch rejects overlapping active signal pins before initialization.

## Validation scope

The actual driver sources passed host-side checks using mocked devices and platform APIs: 640 sensing assertions across ESP32/S3 variants, 111 microphone assertions, and 98 camera assertions. The sketch and public headers also passed eight mocked syntax/API configurations. The capture utility passed eight success/failure cases.

A subsequent Arduino IDE log supplied by the developer shows successful compilation of the selected sketch configuration. Compilation does not establish successful upload, correct wiring, sensor accuracy, DMA behavior, or real-time behavior on the final robot. The camera and complete hardware integration remain unverified on physical hardware.
