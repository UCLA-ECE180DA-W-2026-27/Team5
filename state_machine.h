#pragma once
#include <stdint.h>

// All calls belong to ONE main loop. Other tasks must queue their events.
// Callbacks submit requests and return immediately; never block or re-enter.
typedef enum {
  ROBOT_MOVE_STOP, ROBOT_MOVE_FREE_ROAM, ROBOT_MOVE_SIT,
  ROBOT_MOVE_TURN_LEFT, ROBOT_MOVE_TURN_RIGHT
} robot_motion_t;
typedef void (*robot_motion_handler_t)(robot_motion_t motion, uint32_t request_id);
typedef enum {
  ROBOT_EVENT_FLIPPED, ROBOT_EVENT_UPRIGHT,
  ROBOT_EVENT_LIFTED, ROBOT_EVENT_LOWERED,
  ROBOT_EVENT_EDGE, ROBOT_EVENT_EDGE_CLEAR,
  ROBOT_EVENT_ERROR, ROBOT_EVENT_ERROR_CLEAR,
  ROBOT_EVENT_DISCONNECTED, ROBOT_EVENT_CONNECTED,
  ROBOT_EVENT_LOW_BATTERY, ROBOT_EVENT_BATTERY_OK,
  ROBOT_EVENT_CHARGING, ROBOT_EVENT_UNDOCKED,
  ROBOT_EVENT_FACE, ROBOT_EVENT_FACE_LOST,
  ROBOT_EVENT_INSULT, ROBOT_EVENT_APOLOGY,
  ROBOT_EVENT_FACE_LEFT, ROBOT_EVENT_FACE_RIGHT
} robot_event_t;

bool state_machine_init(void);
void state_machine_update(void);
void state_machine_command(const char *line); // Serial bench adapter + event names
void state_machine_event(robot_event_t event);

// Bind recognition results to the Listening window that produced them.
// A token becomes invalid after timeout, safety interruption or consumption.
uint32_t state_machine_begin_listening(void); // zero means currently blocked
bool state_machine_voice(uint32_t session_id, const char *command);
uint32_t state_machine_listening_session(void); // zero if no active window

void state_machine_set_motion_handler(robot_motion_handler_t handler);
void state_machine_motion_done(uint32_t request_id, bool success);
uint32_t state_machine_active_motion(void); // current SIT request, zero otherwise
// Backwards-compatible SIT-only hook; prefer the generic handler above.
void state_machine_set_sit_handler(void (*handler)(void));
// Bench shortcuts default ON for your prototype. Disable for real integration.
// Disabling bench mode does not disable typed sensor events or voice APIs.
void state_machine_enable_bench(bool enabled);
