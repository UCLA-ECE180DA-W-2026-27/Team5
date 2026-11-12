#pragma once
#include <stdint.h>

typedef enum {
    FACE_IDLE,
    FACE_HAPPY,
    FACE_SHY,
    FACE_ANGRY,
    FACE_WAKING,
    FACE_LISTENING,
    FACE_CONFUSED,
    FACE_SITTING,
    FACE_STUDY,
    FACE_LIFTED,
    FACE_FLIPPED,
    FACE_EDGE,
    FACE_LOW_BATTERY,
    FACE_SLEEP,
    FACE_CHARGING,
    FACE_NO_CONNECTION,
    FACE_ERROR,
    FACE_ACK                // appended to preserve existing enum values
} face_t;

bool display_init(void);
void display_set_face(face_t face);
void display_show_timer(uint32_t seconds_left);
void display_update(void);
