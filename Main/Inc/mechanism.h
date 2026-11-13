#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdbool.h>
#include <stdint.h>

void Mechanism_Init(void);
void Camera_SetAngle(uint8_t angle);
uint8_t Camera_GetAngle(void);
uint32_t Camera_PositionWaitMs(uint8_t angle);
void Lift_SetStartPosition(void);
/* Estimate the wait before commanding the given angle. */
uint32_t Lift_PositionWaitMs(uint8_t angle);
void Lift_SetTravelPosition(void);
/* Explicit maintenance preset; no autonomous filter sequence implied. */
void Lift_SetFirstGreenFilterPosition(void);
void Claw_SetSeparationHold(uint8_t extra_close_deg);
void Claw_RestartAction(void);
bool Claw_Open(uint32_t now_ms);
/* Route-only reuse of an already completed ordinary-open pose. */
bool Claw_OpenIfNeeded(uint32_t now_ms);
/* Explicit two-claw physical full-open endpoints, with existing calibration. */
bool Claw_FullOpen(uint32_t now_ms);
bool Claw_OpenLeft(uint32_t now_ms);
bool Claw_OpenRight(uint32_t now_ms);
bool Claw_ClusterOpenLeft(uint32_t now_ms);
bool Claw_ClusterOpenRight(uint32_t now_ms);
bool Claw_SetBumpPosition(bool closed, uint32_t now_ms);
bool Claw_Retract(uint32_t now_ms);
bool Claw_Touch(uint32_t now_ms);
bool Claw_HalfTouch(uint32_t now_ms);

#endif
