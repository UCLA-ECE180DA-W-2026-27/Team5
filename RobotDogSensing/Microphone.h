#pragma once
#include "SensorTypes.h"

// Two INMP441s share BCLK, WS, and DIN. Tie one L/R LOW (left) and the
// other L/R HIGH (right). Their SD pins share DIN: INMP441 tri-states SD
// outside its selected slot. Power both from 3.3 V and use a common ground.
// Call configure/init from setup(), before other tasks call this module.
struct mic_pins_t {
  int bclk;
  int ws;
  int din;
  bool right_channel; // Initial mono output selection; both slots are captured.
};

enum mic_channel_t { MIC_LEFT, MIC_RIGHT, MIC_MIX };

bool mic_configure(const mic_pins_t *pins);
bool mic_init(uint32_t sample_rate_hz);
// Change which buffered channel is emitted by mic_read(), without rewiring.
// MIX is the arithmetic average of left/right, not sound localization.
bool mic_select_channel(mic_channel_t channel);

// Immediate, nonblocking read of buffered signed 16-bit mono PCM. Stereo is
// captured internally; the PDF's mono output format is preserved.
// Returns at most min(max_samples, 320) samples, or valid=false if unavailable.
// There may be four outstanding chunks. Do not modify borrowed samples.
// Copying sample bytes into a consumer-owned buffer is allowed; copying the
// chunk struct does not create a second ownership lease.
// A chunk remains unchanged until mic_release(), including during async sending.
audio_chunk_t mic_read(size_t max_samples);
void mic_release(audio_chunk_t *chunk);

uint32_t mic_sample_rate_hz();
uint32_t mic_dropped_chunks();

// Continuous capture; no wake-word detection, Wi-Fi, or audio playback here.
// t_ms approximates the first sample's MCU millis() time. It is reconstructed
// from sample count and anchored to task read time, not a hardware timestamp.
// Keep t_ms when forwarding audio; drops are gaps, not silence samples.
// Startup-only configuration. Reset the board to change pins/rate or stop I2S.
