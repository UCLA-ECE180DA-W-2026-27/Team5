#include "Camera.h"

#include <driver/gpio.h>
#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {
constexpr uint32_t kMaxFrameAgeMs = 1000;
constexpr uint32_t kSingleBufferRefreshMs = 100;
constexpr uint32_t kWorkerStackBytes = 4096;

portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
camera_pins_t pins = {};
bool configured = false;
bool started = false;
size_t buffer_count = 0;
camera_fb_t *latest = nullptr;
camera_fb_t *borrowed = nullptr;
uint32_t borrowed_lease = 0;
uint32_t next_lease = 0;
uint32_t captured_count = 0;
esp_err_t last_error = ESP_ERR_INVALID_STATE;

void set_error(esp_err_t error) {
  portENTER_CRITICAL(&state_mux);
  last_error = error;
  portEXIT_CRITICAL(&state_mux);
}

uint32_t frame_time_ms(const camera_fb_t *fb) {
  // esp_camera timestamps the first DMA buffer, in time since MCU boot.
  const uint64_t usec = static_cast<uint64_t>(fb->timestamp.tv_sec) *
                            1000000ULL +
                        static_cast<uint64_t>(fb->timestamp.tv_usec);
  return static_cast<uint32_t>(usec / 1000ULL);
}

bool pins_valid(const camera_pins_t &p) {
  const int inputs[] = {p.d0, p.d1, p.d2, p.d3, p.d4, p.d5,
                        p.d6, p.d7, p.vsync, p.href, p.pclk};
  const int outputs[] = {p.xclk, p.sda, p.scl};
  for (int pin : inputs) {
    if (!GPIO_IS_VALID_GPIO(pin)) return false;
  }
  for (int pin : outputs) {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) return false;
  }
  if (p.pwdn != -1 && !GPIO_IS_VALID_OUTPUT_GPIO(p.pwdn)) return false;
  if (p.reset != -1 && !GPIO_IS_VALID_OUTPUT_GPIO(p.reset)) return false;
  const int all[] = {p.pwdn, p.reset, p.xclk, p.sda, p.scl,
                     p.d0, p.d1, p.d2, p.d3, p.d4, p.d5, p.d6, p.d7,
                     p.vsync, p.href, p.pclk};
  for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); ++i) {
    if (all[i] == -1) continue;
    for (size_t j = 0; j < i; ++j) {
      if (all[i] == all[j]) return false;
    }
  }
  // Board-specific flash/PSRAM, USB, boot-strap and other module conflicts
  // must additionally be checked in the project's pin configuration.
  return true;
}

bool frame_size(uint16_t width, uint16_t height, framesize_t *out) {
  struct Size { uint16_t w; uint16_t h; framesize_t size; };
  static const Size sizes[] = {
      {160, 120, FRAMESIZE_QQVGA}, {320, 240, FRAMESIZE_QVGA},
      {640, 480, FRAMESIZE_VGA}, {800, 600, FRAMESIZE_SVGA},
      {1024, 768, FRAMESIZE_XGA}, {1280, 1024, FRAMESIZE_SXGA},
      {1600, 1200, FRAMESIZE_UXGA}};
  for (const Size &s : sizes) {
    if (s.w == width && s.h == height) {
      *out = s.size;
      return true;
    }
  }
  return false;
}

void capture_task(void *) {
  for (;;) {
    camera_fb_t *retired = nullptr;
    bool can_capture = false;
    const uint32_t now_ms = millis();
    portENTER_CRITICAL(&state_mux);
    if (buffer_count == 2) {
      // Never ask for a third framebuffer while two are held. The capture
      // driver itself owns at most two buffers; no JPEG copies are made.
      can_capture = !(borrowed && latest);
    } else if (!borrowed) {
      // With one buffer, recycle the cache periodically to obtain a new
      // exposure. During capture get_frame() simply reports not-ready.
      if (!latest || uint32_t(now_ms - frame_time_ms(latest)) >=
                         kSingleBufferRefreshMs) {
        retired = latest;
        latest = nullptr;
        can_capture = true;
      }
    }
    portEXIT_CRITICAL(&state_mux);
    if (retired) esp_camera_fb_return(retired);
    if (!can_capture) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    // The vendor API waits for DMA. Only this worker ever calls it;
    // behavior and wireless callers use our immediate cached getter.
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      set_error(ESP_ERR_TIMEOUT);
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (fb->format != PIXFORMAT_JPEG || !fb->buf || fb->len < 4 ||
        fb->buf[0] != 0xFF || fb->buf[1] != 0xD8) {
      esp_camera_fb_return(fb);
      set_error(ESP_ERR_INVALID_RESPONSE);
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    portENTER_CRITICAL(&state_mux);
    retired = latest;
    latest = fb;
    ++captured_count;
    last_error = ESP_OK;
    portEXIT_CRITICAL(&state_mux);
    if (retired) esp_camera_fb_return(retired);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
}  // namespace

bool camera_configure(const camera_pins_t *new_pins) {
  // Initialization/configuration are setup-only and single-caller APIs.
  if (started) {
    set_error(ESP_ERR_INVALID_STATE);
    return false;
  }
  if (!new_pins || !pins_valid(*new_pins)) {
    configured = false;
    set_error(ESP_ERR_INVALID_ARG);
    return false;
  }
  pins = *new_pins;
  configured = true;
  return true;
}

bool camera_init(uint16_t width, uint16_t height, uint8_t jpeg_quality) {
  if (started) {
    // Do not spawn duplicate workers or reinitialize a live peripheral.
    set_error(ESP_ERR_INVALID_STATE);
    return false;
  }
#if !defined(CONFIG_SCCB_HARDWARE_I2C_PORT1) || !CONFIG_SCCB_HARDWARE_I2C_PORT1
  // This package owns Wire/I2C0 for sensing. The bundled esp32-camera
  // chooses its SCCB controller at build time when SCCB pins are supplied.
  // Merely assigning cfg.sccb_i2c_port does not move that internal bus.
  set_error(ESP_ERR_NOT_SUPPORTED);
  return false;
#endif
  framesize_t size = FRAMESIZE_QVGA;
  if (!configured || jpeg_quality > 63 || !frame_size(width, height, &size)) {
    set_error(ESP_ERR_INVALID_ARG);
    return false;
  }
  const bool has_psram = psramFound();
  if (!has_psram && (width > 320 || height > 240)) {
    set_error(ESP_ERR_NOT_SUPPORTED);
    return false;
  }

  camera_config_t cfg = {};
  cfg.pin_pwdn = pins.pwdn;
  cfg.pin_reset = pins.reset;
  cfg.pin_xclk = pins.xclk;
  cfg.pin_sccb_sda = pins.sda;
  cfg.pin_sccb_scl = pins.scl;
  cfg.pin_d0 = pins.d0;
  cfg.pin_d1 = pins.d1;
  cfg.pin_d2 = pins.d2;
  cfg.pin_d3 = pins.d3;
  cfg.pin_d4 = pins.d4;
  cfg.pin_d5 = pins.d5;
  cfg.pin_d6 = pins.d6;
  cfg.pin_d7 = pins.d7;
  cfg.pin_vsync = pins.vsync;
  cfg.pin_href = pins.href;
  cfg.pin_pclk = pins.pclk;
  cfg.xclk_freq_hz = 20000000;
  cfg.ledc_timer = LEDC_TIMER_0;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size = size;
  cfg.jpeg_quality = jpeg_quality;
  cfg.fb_count = has_psram ? 2 : 1;
  cfg.fb_location = has_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  // Used by the vendor API only for an externally configured SCCB bus;
  // the build-time CONFIG_SCCB_HARDWARE_I2C_PORT1 guard above is essential.
  cfg.sccb_i2c_port = 1;
  const esp_err_t error = esp_camera_init(&cfg);
  if (error != ESP_OK) {
    set_error(error);
    return false;
  }
  buffer_count = cfg.fb_count;
  started = true;
  TaskHandle_t task = nullptr;
  if (xTaskCreate(capture_task, "camera_capture", kWorkerStackBytes,
                  nullptr, 1, &task) != pdPASS) {
    esp_camera_deinit();
    started = false;
    buffer_count = 0;
    set_error(ESP_ERR_NO_MEM);
    return false;
  }
  set_error(ESP_OK);
  return true;
}

camera_frame_t camera_get_frame(void) {
  camera_frame_t out = {};
  camera_fb_t *expired = nullptr;
  const uint32_t now_ms = millis();
  portENTER_CRITICAL(&state_mux);
  if (!borrowed && latest) {
    const uint32_t captured_ms = frame_time_ms(latest);
    if (uint32_t(now_ms - captured_ms) > kMaxFrameAgeMs) {
      expired = latest;
      latest = nullptr;
    } else {
      borrowed = latest;
      latest = nullptr;
      if (++next_lease == 0) ++next_lease;
      borrowed_lease = next_lease;
      out = {borrowed->buf, borrowed->len,
             static_cast<uint16_t>(borrowed->width),
             static_cast<uint16_t>(borrowed->height), captured_ms,
             true, borrowed_lease};
    }
  }
  portEXIT_CRITICAL(&state_mux);
  if (expired) esp_camera_fb_return(expired);
  return out;
}

void camera_release_frame(camera_frame_t *frame) {
  if (!frame) return;
  camera_fb_t *released = nullptr;
  portENTER_CRITICAL(&state_mux);
  if (borrowed && frame->lease != 0 && frame->lease == borrowed_lease &&
      frame->data == borrowed->buf) {
    released = borrowed;
    borrowed = nullptr;
    borrowed_lease = 0;
  }
  portEXIT_CRITICAL(&state_mux);
  if (released) esp_camera_fb_return(released);
  *frame = {};
}

uint32_t camera_captured_frames(void) {
  portENTER_CRITICAL(&state_mux);
  const uint32_t count = captured_count;
  portEXIT_CRITICAL(&state_mux);
  return count;
}

esp_err_t camera_last_error(void) {
  portENTER_CRITICAL(&state_mux);
  const esp_err_t error = last_error;
  portEXIT_CRITICAL(&state_mux);
  return error;
}
