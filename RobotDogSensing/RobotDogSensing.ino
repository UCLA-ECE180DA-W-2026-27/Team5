#include <Arduino.h>
#include <math.h>
#include "DriverConfig.h"
#include "Sensing.h"

static bool running = false;
static bool binary_transfer = false;
static uint32_t audio_samples_seen = 0;
static uint32_t audio_last_ms = 0;
static int audio_peak = 0;
static double audio_rms = 0;

static void remember_audio(const audio_chunk_t &chunk) {
  if (!chunk.valid || !chunk.n_samples) return;
  int peak = 0;
  double sum = 0;
  for (size_t i = 0; i < chunk.n_samples; ++i) {
    const int value = chunk.samples[i];
    const int magnitude = value < 0 ? -value : value;
    if (magnitude > peak) peak = magnitude;
    sum += double(value) * value;
  }
  audio_samples_seen += chunk.n_samples;
  audio_last_ms = chunk.t_ms;
  audio_peak = peak;
  audio_rms = sqrt(sum / chunk.n_samples);
}

// Reject overlapping pins BEFORE initializing any hardware in the test sketch.
// This is a basic check; board-specific flash/PSRAM/boot-pin restrictions still apply.
static bool check_test_pins() {
  int used[32];
  size_t n = 0;
  auto add = [&](int pin, bool optional = false) {
    if (pin < 0) return optional;
    for (size_t i = 0; i < n; ++i) {
      if (used[i] == pin) {
        Serial.printf("ERROR: GPIO %d is used by two active signals. Edit DriverConfig.h.\n", pin);
        return false;
      }
    }
    used[n++] = pin;
    return true;
  };
  if (TEST_SENSING) {
    if (!add(SENSING_PINS.sda) || !add(SENSING_PINS.scl) ||
        !add(SENSING_PINS.front_xshut) || !add(SENSING_PINS.down_xshut)) return false;
  }
  if (TEST_MIC) {
    if (!add(MIC_PINS.bclk) || !add(MIC_PINS.ws) || !add(MIC_PINS.din)) return false;
  }
  if (TEST_CAMERA) {
    if (!CAMERA_PIN_MAP_CONFIRMED) {
      Serial.println("ERROR: camera defaults are AI-Thinker ESP32-CAM only, not an S3 pin map.");
      Serial.println("Camera testing is optional. See DriverConfig.h before enabling it on S3.");
      return false;
    }
    const int required[] = {CAMERA_PINS.xclk, CAMERA_PINS.sda, CAMERA_PINS.scl,
      CAMERA_PINS.d0, CAMERA_PINS.d1, CAMERA_PINS.d2, CAMERA_PINS.d3,
      CAMERA_PINS.d4, CAMERA_PINS.d5, CAMERA_PINS.d6, CAMERA_PINS.d7,
      CAMERA_PINS.vsync, CAMERA_PINS.href, CAMERA_PINS.pclk};
    for (int pin : required) {
      if (pin < 0) {
        Serial.println("ERROR: camera pin map is unset. Fill CAMERA_PINS for your exact board.");
        return false;
      }
      if (!add(pin)) return false;
    }
    if (!add(CAMERA_PINS.pwdn, true) || !add(CAMERA_PINS.reset, true)) return false;
  }
  return true;
}

static size_t write_binary(const uint8_t *bytes, size_t len) {
  size_t sent = 0;
  uint32_t last_progress_ms = millis();
  while (sent < len) {
    const size_t part = len - sent > 512 ? 512 : len - sent;
    const size_t written = Serial.write(bytes + sent, part);
    if (written) { sent += written; last_progress_ms = millis(); }
    if (uint32_t(millis() - last_progress_ms) > 3000) return sent;
    delay(1); // Test-only transfer; acquisition continues in background tasks.
  }
  return sent;
}

static void capture_audio(uint32_t seconds) {
  if (!TEST_MIC || seconds < 1 || seconds > 10) {
    Serial.println("ERROR: use mode 2/4 and A1 through A10.");
    return;
  }
  // Drain buffered older audio before recording a new test clip.
  for (unsigned i = 0; i < 64; ++i) {
    audio_chunk_t old = mic_read(320);
    if (!old.valid) break;
    mic_release(&old);
  }
  const uint32_t total_samples = MIC_SAMPLE_RATE_HZ * seconds;
  uint32_t sent_samples = 0;
  uint32_t sent_bytes = 0;
  uint32_t last_data_ms = millis();
  const uint32_t dropped_before = mic_dropped_chunks();
  Serial.printf("PCM_BEGIN %lu %lu\n", (unsigned long)(total_samples * 2),
                (unsigned long)MIC_SAMPLE_RATE_HZ);
  binary_transfer = true;
  while (sent_samples < total_samples) {
    size_t wanted = total_samples - sent_samples;
    if (wanted > 320) wanted = 320;
    audio_chunk_t chunk = mic_read(wanted);
    if (!chunk.valid) {
      if (uint32_t(millis() - last_data_ms) > 2000) break;
      delay(1);
      continue;
    }
    const size_t wanted_bytes = chunk.n_samples * sizeof(int16_t);
    const size_t written = write_binary(reinterpret_cast<const uint8_t *>(chunk.samples),
                                        wanted_bytes);
    sent_samples += written / sizeof(int16_t);
    sent_bytes += written;
    last_data_ms = millis();
    mic_release(&chunk); // The synchronous serial write has finished using the data.
    if (written != wanted_bytes) break;
  }
  binary_transfer = false;
  Serial.printf("\nPCM_END %lu dropped=%lu\n", (unsigned long)sent_bytes,
    (unsigned long)(mic_dropped_chunks() - dropped_before));
}

static void capture_jpeg() {
  if (!TEST_CAMERA) { Serial.println("ERROR: camera requires mode 3/4."); return; }
  camera_frame_t frame = camera_get_frame();
  if (!frame.valid) { Serial.println("ERROR: no fresh camera frame yet."); return; }
  Serial.printf("JPEG_BEGIN %u %u %u\n", (unsigned)frame.len, frame.width, frame.height);
  binary_transfer = true;
  const bool ok = write_binary(frame.data, frame.len) == frame.len;
  binary_transfer = false;
  camera_release_frame(&frame); // Release only after the consumer finishes.
  Serial.println(ok ? "\nJPEG_END" : "\nJPEG_ERROR");
}

void setup() {
  Serial.begin(TEST_SERIAL_BAUD);
  const uint32_t start_ms = millis();
  while (!Serial && uint32_t(millis() - start_ms) < 2000) delay(10);
  Serial.printf("\nRobotDogSensing test mode=%d\n", DRIVER_TEST_MODE);
  if (!check_test_pins()) return;
  bool ok = true;
  if (TEST_SENSING) {
    const init_status_t s = sensing_init(&SENSING_PINS);
    Serial.printf("INIT I2C=%d IMU=%d FRONT=%d DOWN=%d\n", s.i2c, s.imu, s.front_tof, s.down_tof);
    // Keep testing the devices that did initialize, even if one is missing.
    ok = ok && s.i2c;
  }
  if (TEST_MIC) {
    const bool mic_ok = mic_configure(&MIC_PINS) && mic_init(MIC_SAMPLE_RATE_HZ);
    Serial.printf("INIT MIC=%d\n", mic_ok);
    ok = ok && mic_ok;
  }
  if (TEST_CAMERA) {
    const bool cam_ok = camera_configure(&CAMERA_PINS) &&
      camera_init(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_JPEG_QUALITY);
    Serial.printf("INIT CAMERA=%d\n", cam_ok);
    ok = ok && cam_ok;
  }
  running = ok;
  Serial.println("Commands: A5 = 5 s PCM; L/R/M = left/right/mix mic; J = JPEG; S = status.");
  Serial.println(running ? "READY" : "ERROR: inspect INIT results and wiring.");
}

static void print_status() {
  if (TEST_SENSING) {
    const imu_reading_t imu = imu_read();
    const tof_reading_t front = tof_read(TOF_FRONT);
    const tof_reading_t down = tof_read(TOF_DOWN);
    Serial.printf("IMU valid=%d t=%lu a_g=[%.3f,%.3f,%.3f] w_dps=[%.2f,%.2f,%.2f]\n",
      imu.valid, (unsigned long)imu.t_ms, imu.accel_g[0], imu.accel_g[1], imu.accel_g[2],
      imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2]);
    Serial.printf("TOF front_mm=%u valid=%d t=%lu down_mm=%u valid=%d t=%lu\n",
      front.distance_mm, front.valid, (unsigned long)front.t_ms,
      down.distance_mm, down.valid, (unsigned long)down.t_ms);
  }
  if (TEST_MIC) {
    Serial.printf("MIC total_samples=%lu peak=%d rms=%.1f t=%lu dropped=%lu\n",
      (unsigned long)audio_samples_seen, audio_peak, audio_rms,
      (unsigned long)audio_last_ms, (unsigned long)mic_dropped_chunks());
  }
  if (TEST_CAMERA) Serial.printf("CAMERA frames=%lu\n", (unsigned long)camera_captured_frames());
}

void loop() {
  if (!running) { delay(10); return; }
  if (TEST_SENSING) sensing_update();
  static char line[16];
  static size_t used = 0;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (!used) continue;
      line[used] = '\0'; used = 0;
      if (line[0] == 'A' || line[0] == 'a') capture_audio(strtoul(line + 1, nullptr, 10));
      else if (line[0] == 'J' || line[0] == 'j') capture_jpeg();
      else if (line[0] == 'S' || line[0] == 's') print_status();
      else if (TEST_MIC && (line[0] == 'L' || line[0] == 'l')) {
        mic_select_channel(MIC_LEFT); Serial.println("CHANNEL LEFT");
      }
      else if (TEST_MIC && (line[0] == 'R' || line[0] == 'r')) {
        mic_select_channel(MIC_RIGHT); Serial.println("CHANNEL RIGHT");
      }
      else if (TEST_MIC && (line[0] == 'M' || line[0] == 'm')) {
        mic_select_channel(MIC_MIX); Serial.println("CHANNEL MIX");
      }
      else Serial.println("ERROR: unknown command.");
    } else if (used + 1 < sizeof(line)) line[used++] = c;
  }
  static uint32_t last_print_ms = 0;
  if (!binary_transfer && uint32_t(millis() - last_print_ms) >= 1000) {
    last_print_ms = millis();
    print_status();
  }
  // Drain the audio queue between status lines, like a real streaming consumer.
  if (TEST_MIC) {
    audio_chunk_t chunk = mic_read(320);
    if (chunk.valid) { remember_audio(chunk); mic_release(&chunk); }
  }
  delay(1);
}
