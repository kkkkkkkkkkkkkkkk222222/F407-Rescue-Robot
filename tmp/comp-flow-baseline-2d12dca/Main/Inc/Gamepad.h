#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  GAMEPAD_IDLE = 0,
  GAMEPAD_RUNNING,
  GAMEPAD_TIMEOUT,
  GAMEPAD_FAULT,
  GAMEPAD_STOPPED
} GamepadState;

typedef struct {
  GamepadState state;
  int16_t forward_mm_s;
  int16_t left_mm_s;
  int16_t yaw_mm_s;
  uint8_t camera_angle;
  uint8_t lift_angle;
  uint8_t left_claw_angle;
  uint8_t right_claw_angle;
  uint8_t buttons;
  uint8_t command_sequence;
  bool received;
  bool armed;
} GamepadStatus;

void Gamepad_Init(uint32_t now_ms);
void Gamepad_Process(uint32_t now_ms);
GamepadStatus Gamepad_GetStatus(void);

#endif
