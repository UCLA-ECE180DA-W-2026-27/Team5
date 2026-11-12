#pragma once
#include <stdint.h>

typedef enum {
    SOUND_BARK,
    SOUND_BARK_SHORT,
    SOUND_CONFUSED,
    SOUND_ACK,
    SOUND_TIMER_DONE,
    SOUND_LOW_BATTERY,
    SOUND_WAKE,
    SOUND_HELLO,
    SOUND_LISTENING,
    SOUND_STUDY_START,
    SOUND_PAUSE,
    SOUND_RESUME,
    SOUND_CHARGING,
    SOUND_FLIPPED,
    SOUND_UPRIGHT,
    SOUND_ANGRY,
    SOUND_FORGIVE,
    SOUND_HAPPY
} sound_t;

bool speaker_init(void);
void speaker_play(sound_t sound);
void speaker_stop(void);
bool speaker_is_playing(void);
void speaker_set_volume(uint8_t volume_pct);
