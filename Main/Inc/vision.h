#ifndef VISION_H
#define VISION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VISION_FRAME_HEAD_1    0xA3U
#define VISION_FRAME_HEAD_2    0xB3U
#define VISION_FRAME_TAIL      0xC3U
#define VISION_STOP_VALUE      0xFFFFU
#define VISION_NO_TARGET_VALUE 0xFFFEU

typedef struct {
  uint16_t x;
  uint16_t y;
  uint32_t tick_ms;
  bool valid;
  bool stop;
} VisionData;

void Vision_Init(void);
void Vision_ResetParser(void);
void Vision_ParseBytes(const uint8_t *data, size_t size, uint32_t tick_ms);
VisionData Vision_GetSnapshot(void);
bool Vision_IsFresh(const VisionData *data, uint32_t now_ms, uint32_t timeout_ms);

#endif
