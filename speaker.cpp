#include "speaker.h"
#include <Arduino.h>
#include <ESP_I2S.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <math.h>
#include "audio_data.h"

namespace {
I2SClass audio;
TaskHandle_t audioTask=nullptr;
bool ready=false;
const int SAMPLE_RATE=16000, BLOCK=256;
std::atomic<uint8_t> volume{10};
// High 16 bits: request generation. Low 16 bits: clip index + 1; zero = stop.
std::atomic<uint32_t> request{0}, finished{0};
const uint8_t SOUND_CLIPS[]={5,18,17,16,2,11,4,12,10,7,6,8,9,1,13,14,15,3};

int16_t clipSample(uint32_t frame, int clip, uint8_t gain) {
  if (frame>=CLIP_LENGTHS[clip]) return 0;
  int source=clip;
  if (frame>=CLIP_HEAD_LENGTHS[source]) {
    frame-=CLIP_HEAD_LENGTHS[source];
    if (clip==15) { if (frame<1920) return 0; frame-=1920; }
    source=CLIP_TAIL_INDEX[source];
    if (source<0 || frame>=CLIP_HEAD_LENGTHS[source]) return 0;
  }
  return (int16_t)((int32_t)CLIP_DATA[source][frame]*gain/100);
}
void worker(void*) {
  int16_t samples[BLOCK*2];
  uint32_t seen=0, frame=0, count=0;
  int clip=0;
  while (true) {
    uint32_t next=request.load();
    if (next!=seen) {
      seen=next; frame=0;
      uint16_t code=(uint16_t)seen;
      count=0;
      if (code>0 && code<=sizeof(SOUND_CLIPS)) {
        sound_t sound=(sound_t)(code-1);
        clip=SOUND_CLIPS[sound]; count=CLIP_LENGTHS[clip];
      }
    }
    uint8_t gain=volume.load();
    for (int i=0;i<BLOCK;i++) {
      int16_t value=0;
      if (frame<count) {
        value=clipSample(frame,clip,gain);
        frame++;
      }
      samples[2*i]=samples[2*i+1]=value;
    }
    size_t sent=0;
    while (sent<sizeof(samples)) {
      size_t n=audio.write(reinterpret_cast<uint8_t*>(samples)+sent,sizeof(samples)-sent);
      if (!n) { finished.store(seen); frame=count; break; }
      sent+=n;
    }
    if (frame>=count) finished.store(seen);
    vTaskDelay(1);
  }
}
void submit(uint16_t code) {
  uint32_t old=request.load(), next;
  do { next=((old+0x10000UL)&0xffff0000UL)|code; }
  while (!request.compare_exchange_weak(old,next));
}
} // namespace

bool speaker_init(void) {
  if (ready) return true;
  audio.setPins(4,5,6);
  if (!audio.begin(I2S_MODE_STD,SAMPLE_RATE,I2S_DATA_BIT_WIDTH_16BIT,I2S_SLOT_MODE_STEREO)) return false;
  if (xTaskCreate(worker,"Speaker",4096,nullptr,1,&audioTask)!=pdPASS) {
    audio.end(); return false;
  }
  ready=true; return true;
}
void speaker_play(sound_t sound) {
  if (!ready || sound<SOUND_BARK || sound>SOUND_HAPPY) return;
  submit((uint16_t)sound+1);
}
void speaker_stop(void) { if (ready) submit(0); }
bool speaker_is_playing(void) {
  uint32_t r=request.load();
  return ready && (uint16_t)r!=0 && finished.load()!=r;
}
void speaker_set_volume(uint8_t pct) { volume.store(pct>100 ? 100 : pct); }
