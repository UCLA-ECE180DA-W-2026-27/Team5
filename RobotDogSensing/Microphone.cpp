#include "Microphone.h"

#if !defined(ARDUINO_ARCH_ESP32)
#error "Microphone requires Arduino-ESP32 3.x (ESP-IDF 5)."
#endif

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <soc/soc_caps.h>

#if SOC_I2S_NUM < 2
#error "This driver reserves I2S1 for audio and requires ESP32 or ESP32-S3."
#endif

namespace {
constexpr size_t kBlockSamples = 320;
constexpr size_t kPendingBlocks = 8;
constexpr size_t kLeaseCount = 4;
constexpr uint32_t kReadTimeoutMs = 100;

struct PendingBlock {
  int16_t samples[kBlockSamples * 2]; // Interleaved left/right stereo frames.
  size_t count;
  uint64_t first_us;
};
struct BorrowedBlock {
  int16_t samples[kBlockSamples];
  uint32_t token;
};

mic_pins_t s_pins = {-1, -1, -1, false};
bool s_configured = false;
bool s_initialized = false;
uint32_t s_rate_hz = 0;
mic_channel_t s_channel = MIC_LEFT;
i2s_chan_handle_t s_rx = nullptr;
TaskHandle_t s_task = nullptr;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
PendingBlock s_pending[kPendingBlocks] = {};
BorrowedBlock s_borrowed[kLeaseCount] = {};
size_t s_head = 0;
size_t s_count = 0;
size_t s_read_offset = 0;
uint32_t s_next_token = 0;
uint32_t s_dropped = 0;
bool s_dma_gap = false;

bool pins_equal(const mic_pins_t &a, const mic_pins_t &b) {
  return a.bclk == b.bclk && a.ws == b.ws && a.din == b.din &&
         a.right_channel == b.right_channel;
}

bool pins_valid(const mic_pins_t &pins) {
  return GPIO_IS_VALID_OUTPUT_GPIO(pins.bclk) &&
         GPIO_IS_VALID_OUTPUT_GPIO(pins.ws) && GPIO_IS_VALID_GPIO(pins.din) &&
         pins.bclk != pins.ws && pins.bclk != pins.din &&
         pins.ws != pins.din;
}

bool IRAM_ATTR on_dma_overflow(i2s_chan_handle_t,
                              i2s_event_data_t *, void *) {
  portENTER_CRITICAL_ISR(&s_mux);
  ++s_dropped;
  s_dma_gap = true;
  portEXIT_CRITICAL_ISR(&s_mux);
  return false;
}

void capture_task(void *) {
  int32_t words[kBlockSamples * 2];
  int16_t pcm[kBlockSamples * 2];
  uint64_t next_us = 0;
  // Keep fractional microseconds across blocks for non-divisible sample rates.
  uint64_t fraction = 0;
  for (;;) {
    size_t received_bytes = 0;
    const esp_err_t error = i2s_channel_read(
        s_rx, words, sizeof(words), &received_bytes, kReadTimeoutMs);
    const size_t samples = received_bytes / (2 * sizeof(words[0]));
    if (samples == 0 || (error != ESP_OK && error != ESP_ERR_TIMEOUT)) {
      next_us = 0;
      fraction = 0;
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    bool had_gap;
    portENTER_CRITICAL(&s_mux);
    had_gap = s_dma_gap;
    s_dma_gap = false;
    portEXIT_CRITICAL(&s_mux);
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t block_us = samples * 1000000ULL / s_rate_hz;
    if (next_us == 0 || had_gap) {
      next_us = now_us > block_us ? now_us - block_us : 0;
      fraction = 0;
    }
    const uint64_t first_us = next_us;
    const uint64_t numerator = samples * 1000000ULL + fraction;
    next_us += numerator / s_rate_hz;
    fraction = numerator % s_rate_hz;

    // INMP441 24-bit signed sample occupies bits 31:8 of each 32-bit slot.
    // Keeping bits 31:16 produces normal signed 16-bit PCM without gain/clipping.
    for (size_t i = 0; i < samples * 2; ++i) {
      pcm[i] = static_cast<int16_t>(words[i] >> 16);
    }

    // All critical sections contain bounded RAM work (<=1280-byte copy or
    // conversion of <=320 stereo frames).
    // DMA reads, peripheral calls, and timestamp arithmetic stay outside them.
    portENTER_CRITICAL(&s_mux);
    if (s_count == kPendingBlocks) {
      s_head = (s_head + 1) % kPendingBlocks;
      --s_count;
      s_read_offset = 0;
      ++s_dropped;
    }
    PendingBlock &dest = s_pending[(s_head + s_count) % kPendingBlocks];
    memcpy(dest.samples, pcm, samples * 2 * sizeof(pcm[0]));
    dest.count = samples;
    dest.first_us = first_us;
    ++s_count;
    portEXIT_CRITICAL(&s_mux);
  }
}

void clean_failed_init(bool enabled) {
  if (s_rx != nullptr) {
    if (enabled) i2s_channel_disable(s_rx);
    i2s_del_channel(s_rx);
    s_rx = nullptr;
  }
  s_rate_hz = 0;
}
}  // namespace

bool mic_configure(const mic_pins_t *pins) {
  if (pins == nullptr || !pins_valid(*pins)) return false;
  if (s_initialized) return pins_equal(s_pins, *pins);
  s_pins = *pins;
  s_channel = pins->right_channel ? MIC_RIGHT : MIC_LEFT;
  s_configured = true;
  return true;
}

bool mic_init(uint32_t sample_rate_hz) {
  if (s_initialized) return sample_rate_hz == s_rate_hz;
  if (!s_configured || sample_rate_hz < 8000 || sample_rate_hz > 48000) {
    return false;
  }

  // The original ESP32 camera uses I2S0; audio explicitly uses I2S1.
  i2s_chan_config_t chan =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chan.dma_desc_num = 6;
  chan.dma_frame_num = kBlockSamples;
  if (i2s_new_channel(&chan, nullptr, &s_rx) != ESP_OK) return false;

  i2s_std_config_t config = {};
  config.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz);
  config.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  config.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
  config.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  config.gpio_cfg.bclk = static_cast<gpio_num_t>(s_pins.bclk);
  config.gpio_cfg.ws = static_cast<gpio_num_t>(s_pins.ws);
  config.gpio_cfg.dout = I2S_GPIO_UNUSED;
  config.gpio_cfg.din = static_cast<gpio_num_t>(s_pins.din);
  if (i2s_channel_init_std_mode(s_rx, &config) != ESP_OK) {
    clean_failed_init(false);
    return false;
  }
  i2s_event_callbacks_t callbacks = {};
  callbacks.on_recv_q_ovf = on_dma_overflow;
  if (i2s_channel_register_event_callback(s_rx, &callbacks, nullptr) != ESP_OK) {
    clean_failed_init(false);
    return false;
  }

  s_rate_hz = sample_rate_hz;
  s_head = 0;
  s_count = 0;
  s_read_offset = 0;
  s_dropped = 0;
  s_dma_gap = false;
  if (i2s_channel_enable(s_rx) != ESP_OK) {
    clean_failed_init(false);
    return false;
  }
  if (xTaskCreate(capture_task, "mic_capture", 8192, nullptr, 2, &s_task) !=
      pdPASS) {
    clean_failed_init(true);
    return false;
  }
  s_initialized = true;
  return true;
}

bool mic_select_channel(mic_channel_t channel) {
  if (channel != MIC_LEFT && channel != MIC_RIGHT && channel != MIC_MIX) {
    return false;
  }
  portENTER_CRITICAL(&s_mux);
  s_channel = channel;
  portEXIT_CRITICAL(&s_mux);
  return true;
}

audio_chunk_t mic_read(size_t max_samples) {
  audio_chunk_t result = {};
  if (!s_initialized || max_samples == 0) return result;
  portENTER_CRITICAL(&s_mux);
  size_t index = kLeaseCount;
  for (size_t i = 0; i < kLeaseCount; ++i) {
    if (s_borrowed[i].token == 0) {
      index = i;
      break;
    }
  }
  if (s_count != 0 && index != kLeaseCount) {
    const PendingBlock &src = s_pending[s_head];
    const size_t remaining = src.count - s_read_offset;
    const size_t count = max_samples < remaining ? max_samples : remaining;
    BorrowedBlock &dest = s_borrowed[index];
    for (size_t i = 0; i < count; ++i) {
      const size_t offset = (s_read_offset + i) * 2;
      const int32_t left = src.samples[offset];
      const int32_t right = src.samples[offset + 1];
      dest.samples[i] = static_cast<int16_t>(
          s_channel == MIC_LEFT ? left :
          s_channel == MIC_RIGHT ? right : (left + right) / 2);
    }
    do { ++s_next_token; } while (s_next_token == 0);
    dest.token = s_next_token;
    result.samples = dest.samples;
    result.n_samples = count;
    result.t_ms = static_cast<uint32_t>(
        (src.first_us + s_read_offset * 1000000ULL / s_rate_hz) / 1000ULL);
    result.valid = true;
    result.lease = dest.token;
    s_read_offset += count;
    if (s_read_offset == src.count) {
      s_head = (s_head + 1) % kPendingBlocks;
      --s_count;
      s_read_offset = 0;
    }
  }
  portEXIT_CRITICAL(&s_mux);
  return result;
}

void mic_release(audio_chunk_t *chunk) {
  if (chunk == nullptr) return;
  portENTER_CRITICAL(&s_mux);
  for (size_t i = 0; i < kLeaseCount; ++i) {
    if (chunk->lease != 0 && s_borrowed[i].token == chunk->lease &&
        s_borrowed[i].samples == chunk->samples) {
      s_borrowed[i].token = 0;
      break;
    }
  }
  portEXIT_CRITICAL(&s_mux);
  *chunk = {};
}

uint32_t mic_sample_rate_hz() { return s_rate_hz; }

uint32_t mic_dropped_chunks() {
  portENTER_CRITICAL(&s_mux);
  const uint32_t value = s_dropped;
  portEXIT_CRITICAL(&s_mux);
  return value;
}
