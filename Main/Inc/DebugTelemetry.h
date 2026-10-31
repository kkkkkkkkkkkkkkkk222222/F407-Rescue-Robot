#ifndef DEBUG_TELEMETRY_H
#define DEBUG_TELEMETRY_H
#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"
#define DEBUG_TIMED_LINK (APP_BOARD_F405_ROBO && APP_ENABLE_MOTION_DEBUG_TASK)
#if DEBUG_TIMED_LINK
void DebugTelemetry_Init(void);
void DebugTelemetry_Process(void);
uint64_t DebugTelemetry_NowUs(void);
void DebugTelemetry_ClockIRQ(void);
bool DebugTelemetry_RxByte(uint8_t value, bool legacy_idle);
void DebugTelemetry_WheelSample(uint64_t begin, uint64_t end,
    const int64_t position[3], const int32_t delta[3], uint32_t bad, bool pending);
void DebugTelemetry_GyroSample(uint8_t type, int64_t yaw, int32_t rate, uint64_t received);
void DebugTelemetry_ReferenceReset(void);
void DebugTelemetry_TxStarted(const uint8_t *data, uint16_t size, uint64_t queued);
void DebugTelemetry_TxCompleted(const uint8_t *data, uint16_t size);
void DebugTelemetry_TxDropped(void);
#endif
#endif
