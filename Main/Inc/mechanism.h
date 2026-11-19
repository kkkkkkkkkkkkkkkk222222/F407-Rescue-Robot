#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdbool.h>
#include <stdint.h>

void Mechanism_Init(void);
void Camera_SetAngle(uint8_t angle);
uint8_t Camera_GetAngle(void);
uint32_t Camera_PositionWaitMs(uint8_t angle);
void Lift_SetStartPosition(void);
void Lift_SetReleaseRaisePosition(void);
/* Estimate the wait before commanding the given angle. */
uint32_t Lift_PositionWaitMs(uint8_t angle);
void Lift_SetTravelPosition(void);
typedef enum {
  LIFT_LOAD_UNKNOWN = 0,
  LIFT_LOAD_MATERIAL,
  LIFT_LOAD_ORANGE
} LiftLoad;
uint8_t Lift_LoadedTravelAngle(LiftLoad load);
void Lift_SetLoadedTravelPosition(LiftLoad load);
/* Explicit maintenance preset; no autonomous filter sequence implied. */
void Lift_SetFirstGreenFilterPosition(void);
void Claw_SetSeparationHold(uint8_t extra_close_deg);
void Claw_SetOrangeSeparationHold(uint8_t base_close_deg);
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
/* Calibrated bench targets use physical opening in hundredths of a degree. */
bool Claw_BeginBenchSeparation(bool keep_left, uint16_t retained_cdeg,
                               int16_t left_trim_us, int16_t right_trim_us, uint16_t close_dps);
bool Claw_RunBenchSeparation(uint32_t now_ms);
bool Claw_SetBumpPosition(bool closed, uint32_t now_ms);
bool Claw_Retract(uint32_t now_ms);
bool Claw_Touch(uint32_t now_ms);
bool Claw_HalfTouch(uint32_t now_ms);

#endif
