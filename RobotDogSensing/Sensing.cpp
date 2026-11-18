#include "Sensing.h"

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

namespace {
constexpr uint32_t kI2cHz = 400000;
constexpr uint16_t kI2cTimeoutMs = 5;
constexpr uint32_t kImuPollMs = 10;
constexpr uint32_t kTofPollMs = 5;
constexpr uint32_t kImuMaxAgeMs = 100;
constexpr uint32_t kTofMaxAgeMs = 200;
constexpr uint8_t kTofAddress[2] = {0x30, 0x31};
constexpr uint16_t kTofPeriodMs = 50;
constexpr uint32_t kTofBudgetUs = 33000;

portMUX_TYPE cache_mux = portMUX_INITIALIZER_UNLOCKED;
imu_reading_t imu_cache = {};
tof_reading_t tof_cache[2] = {};
float gyro_bias_dps[3] = {};
init_status_t status_cache = {};
Adafruit_VL53L0X tof_devices[2];
sensing_pins_t active_pins = {-1, -1, -1, -1};
TaskHandle_t worker_handle = nullptr;
bool init_attempted = false;
uint8_t imu_address = 0;
uint32_t last_imu_poll_ms = 0;
uint32_t last_tof_poll_ms = 0;

bool elapsed(uint32_t now_ms, uint32_t then_ms, uint32_t period_ms) {
  return static_cast<uint32_t>(now_ms - then_ms) >= period_ms;
}

bool output_pin(int pin) {
  if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) return false;
#if CONFIG_IDF_TARGET_ESP32
  // SPI flash pins; other board-specific reservations remain caller's job.
  if (pin >= 6 && pin <= 11) return false;
#elif CONFIG_IDF_TARGET_ESP32S3
  if (pin >= 26 && pin <= 32) return false;
#endif
  return true;
}

bool valid_pins(const sensing_pins_t *pins) {
  if (pins == nullptr) return false;
  const int values[4] = {pins->sda, pins->scl,
                         pins->front_xshut, pins->down_xshut};
  for (unsigned i = 0; i < 4; ++i) {
    if (!output_pin(values[i])) return false;
    for (unsigned j = 0; j < i; ++j) {
      if (values[i] == values[j]) return false;
    }
  }
  return true;
}

bool same_pins(const sensing_pins_t &a, const sensing_pins_t &b) {
  return a.sda == b.sda && a.scl == b.scl &&
         a.front_xshut == b.front_xshut && a.down_xshut == b.down_xshut;
}

bool ping(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool write_reg(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  const bool queued = Wire.write(reg) == 1 && Wire.write(value) == 1;
  return Wire.endTransmission() == 0 && queued;
}

bool read_regs(uint8_t address, uint8_t reg, uint8_t *data, size_t count) {
  Wire.beginTransmission(address);
  const bool queued = Wire.write(reg) == 1;
  if (Wire.endTransmission(false) != 0 || !queued) return false;
  const size_t received = Wire.requestFrom(address, count, true);
  if (received != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < count; ++i) {
    const int value = Wire.read();
    if (value < 0) return false;
    data[i] = static_cast<uint8_t>(value);
  }
  return true;
}

bool check_reg(uint8_t address, uint8_t reg, uint8_t expected) {
  uint8_t value = 0;
  return read_regs(address, reg, &value, 1) && value == expected;
}

void hold_reset(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT_OPEN_DRAIN);
}

void release_reset(int pin) {
  // Release, never drive XSHUT high. The breakout needs its rated pull-up.
  pinMode(pin, INPUT);
}

bool init_imu() {
  const uint8_t candidates[2] = {0x68, 0x69};
  for (uint8_t address : candidates) {
    // WHO_AM_I stays 0x68 when AD0 changes the bus address to 0x69.
    if (check_reg(address, 0x75, 0x68)) {
      imu_address = address;
      break;
    }
  }
  if (imu_address == 0 || !write_reg(imu_address, 0x6B, 0x80)) return false;
  delay(100); // Setup-only device reset.
  if (!write_reg(imu_address, 0x6B, 0x01) ||
      !write_reg(imu_address, 0x6C, 0x00)) return false;
  delay(30);
  // 100 Hz; DLPF 3; +/-500 degrees/s; +/-4 g; no FIFO; DATA_RDY enabled.
  const uint8_t registers[] = {0x1A, 0x19, 0x1B, 0x1C, 0x23, 0x6A, 0x38};
  const uint8_t values[] = {0x03, 0x09, 0x08, 0x08, 0x00, 0x00, 0x01};
  for (unsigned i = 0; i < sizeof(registers); ++i) {
    if (!write_reg(imu_address, registers[i], values[i]) ||
        !check_reg(imu_address, registers[i], values[i])) return false;
  }
  return check_reg(imu_address, 0x6B, 0x01);
}

bool init_tof(unsigned index, int xshut_pin) {
  release_reset(xshut_pin);
  delay(10); // Setup-only boot settling.
  bool ok = check_reg(0x29, 0xC0, 0xEE);
  if (ok) ok = tof_devices[index].begin(kTofAddress[index], false, &Wire);
  // begin() uses the selected Wire instance; verify both identity and address.
  if (ok) ok = check_reg(kTofAddress[index], 0xC0, 0xEE) && !ping(0x29);
  if (ok) ok = tof_devices[index].setMeasurementTimingBudgetMicroSeconds(kTofBudgetUs);
  if (ok) ok = tof_devices[index].startRangeContinuous(kTofPeriodMs);
  if (!ok) hold_reset(xshut_pin);
  return ok;
}

int16_t signed_word(const uint8_t *data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

void invalidate_imu() {
  portENTER_CRITICAL(&cache_mux);
  imu_cache.valid = false;
  portEXIT_CRITICAL(&cache_mux);
}

void invalidate_tof(unsigned index) {
  portENTER_CRITICAL(&cache_mux);
  tof_cache[index].valid = false;
  portEXIT_CRITICAL(&cache_mux);
}

void poll_imu() {
  uint8_t ready = 0;
  if (!read_regs(imu_address, 0x3A, &ready, 1)) {
    invalidate_imu();
    return;
  }
  if ((ready & 1) == 0) return;
  uint8_t raw[14] = {};
  if (!read_regs(imu_address, 0x3B, raw, sizeof(raw))) {
    invalidate_imu();
    return;
  }
  imu_reading_t reading = {};
  reading.t_ms = millis();
  reading.valid = true;
  float bias[3];
  portENTER_CRITICAL(&cache_mux);
  for (unsigned i = 0; i < 3; ++i) bias[i] = gyro_bias_dps[i];
  portEXIT_CRITICAL(&cache_mux);
  for (unsigned i = 0; i < 3; ++i) {
    reading.accel_g[i] = signed_word(raw + i * 2) / 8192.0f;
    reading.gyro_dps[i] = signed_word(raw + 8 + i * 2) / 65.5f - bias[i];
  }
  portENTER_CRITICAL(&cache_mux);
  imu_cache = reading;
  portEXIT_CRITICAL(&cache_mux);
}

void poll_tof(unsigned index) {
  const uint8_t address = kTofAddress[index];
  uint8_t ready = 0;
  // A checked transport read is needed: Adafruit 1.2.5 can hide bus errors.
  if (!read_regs(address, 0x13, &ready, 1)) {
    invalidate_tof(index);
    return;
  }
  if ((ready & 7) == 0) return;
  uint8_t before[12] = {};
  if (!read_regs(address, 0x14, before, sizeof(before))) {
    invalidate_tof(index);
    return;
  }
  VL53L0X_RangingMeasurementData_t measurement = {};
  measurement.RangeStatus = 0xFF;
  const VL53L0X_Error result =
      tof_devices[index].getRangingMeasurement(&measurement, false);
  uint8_t after[12] = {};
  const bool checked = read_regs(address, 0x14, after, sizeof(after));
  const uint16_t checked_mm =
      (static_cast<uint16_t>(after[10]) << 8) | after[11];
  const bool stable = checked && before[0] == after[0] &&
      before[10] == after[10] && before[11] == after[11];
  // Clear with checked Wire rather than relying on unchecked library transport.
  const bool cleared = write_reg(address, 0x0B, 0x01);
  tof_reading_t reading = {};
  reading.t_ms = millis();
  reading.valid = stable && cleared && result == VL53L0X_ERROR_NONE &&
      measurement.RangeStatus == 0 && measurement.RangeMilliMeter == checked_mm &&
      checked_mm != 0 && checked_mm != UINT16_MAX;
  reading.distance_mm = reading.valid ? checked_mm : 0;
  portENTER_CRITICAL(&cache_mux);
  tof_cache[index] = reading;
  portEXIT_CRITICAL(&cache_mux);
}

void expire_cache() {
  const uint32_t now_ms = millis();
  portENTER_CRITICAL(&cache_mux);
  if (elapsed(now_ms, imu_cache.t_ms, kImuMaxAgeMs)) imu_cache.valid = false;
  for (unsigned i = 0; i < 2; ++i) {
    if (elapsed(now_ms, tof_cache[i].t_ms, kTofMaxAgeMs)) tof_cache[i].valid = false;
  }
  portEXIT_CRITICAL(&cache_mux);
}

void worker(void *) {
  const TickType_t wait_ticks = pdMS_TO_TICKS(2) > 0 ? pdMS_TO_TICKS(2) : 1;
  for (;;) {
    const uint32_t now_ms = millis();
    if (status_cache.imu && elapsed(now_ms, last_imu_poll_ms, kImuPollMs)) {
      last_imu_poll_ms = now_ms;
      poll_imu();
    }
    if (elapsed(now_ms, last_tof_poll_ms, kTofPollMs)) {
      last_tof_poll_ms = now_ms;
      if (status_cache.front_tof) poll_tof(0);
      if (status_cache.down_tof) poll_tof(1);
    }
    expire_cache();
    ulTaskNotifyTake(pdTRUE, wait_ticks);
  }
}
} // namespace

init_status_t sensing_init(const sensing_pins_t *pins) {
  if (!valid_pins(pins)) return {};
  if (init_attempted) return same_pins(*pins, active_pins) ? sensing_status() : init_status_t{};
  init_attempted = true;
  active_pins = *pins;
  hold_reset(pins->front_xshut);
  hold_reset(pins->down_xshut);
  delay(10);
  status_cache.i2c = Wire.begin(pins->sda, pins->scl, kI2cHz);
  Wire.setTimeOut(kI2cTimeoutMs);
  if (!status_cache.i2c) return status_cache;
  status_cache.imu = init_imu();
  status_cache.front_tof = init_tof(0, pins->front_xshut);
  status_cache.down_tof = init_tof(1, pins->down_xshut);
  Wire.setTimeOut(kI2cTimeoutMs);
  last_imu_poll_ms = millis() - kImuPollMs;
  last_tof_poll_ms = millis() - kTofPollMs;
  if (status_cache.imu || status_cache.front_tof || status_cache.down_tof) {
    if (xTaskCreate(worker, "sensing", 6144, nullptr, 2, &worker_handle) != pdPASS) {
      worker_handle = nullptr;
      status_cache.imu = false;
      status_cache.front_tof = false;
      status_cache.down_tof = false;
      hold_reset(pins->front_xshut);
      hold_reset(pins->down_xshut);
    }
  }
  return sensing_status();
}

init_status_t sensing_status(void) {
  portENTER_CRITICAL(&cache_mux);
  const init_status_t result = status_cache;
  portEXIT_CRITICAL(&cache_mux);
  return result;
}

void sensing_update(void) {
  if (worker_handle != nullptr) xTaskNotifyGive(worker_handle);
}

imu_reading_t imu_read(void) {
  expire_cache();
  portENTER_CRITICAL(&cache_mux);
  const imu_reading_t result = imu_cache;
  portEXIT_CRITICAL(&cache_mux);
  return result;
}

tof_reading_t tof_read(tof_sensor_t sensor) {
  if (sensor != TOF_FRONT && sensor != TOF_DOWN) return {};
  expire_cache();
  portENTER_CRITICAL(&cache_mux);
  const tof_reading_t result = tof_cache[sensor == TOF_FRONT ? 0 : 1];
  portEXIT_CRITICAL(&cache_mux);
  return result;
}

void imu_set_gyro_bias(float x_dps, float y_dps, float z_dps) {
  if (!isfinite(x_dps) || !isfinite(y_dps) || !isfinite(z_dps)) return;
  portENTER_CRITICAL(&cache_mux);
  gyro_bias_dps[0] = x_dps;
  gyro_bias_dps[1] = y_dps;
  gyro_bias_dps[2] = z_dps;
  portEXIT_CRITICAL(&cache_mux);
}
