#include "vision.h"

#include "app_config.h"
#include "main.h"

static volatile VisionData latest_data;
static uint8_t frame[7];
static uint8_t frame_index;

static void vision_save_frame(uint32_t tick_ms)
{
  const uint16_t x = ((uint16_t)frame[2] << 8) | frame[3];
  const uint16_t y = ((uint16_t)frame[4] << 8) | frame[5];
  const bool stop = (x == VISION_STOP_VALUE) && (y == VISION_STOP_VALUE);
  const bool no_target = (x == VISION_NO_TARGET_VALUE) &&
                         (y == VISION_NO_TARGET_VALUE);
  const bool coordinates_in_range =
      (x <= APP_VISION_MAX_X) && (y <= APP_VISION_MAX_Y);
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  latest_data.x = x;
  latest_data.y = y;
  latest_data.tick_ms = tick_ms;
  latest_data.stop = stop;
  latest_data.valid = !stop && !no_target && coordinates_in_range;
  if (primask == 0U) {
    __enable_irq();
  }
}

static void vision_parse_byte(uint8_t value, uint32_t tick_ms)
{
  if (frame_index == 0U) {
    if (value == VISION_FRAME_HEAD_1) {
      frame[0] = value;
      frame_index = 1U;
    }
    return;
  }

  if (frame_index == 1U) {
    if (value == VISION_FRAME_HEAD_2) {
      frame[1] = value;
      frame_index = 2U;
    } else if (value != VISION_FRAME_HEAD_1) {
      frame_index = 0U;
    }
    return;
  }

  frame[frame_index++] = value;
  if (frame_index == sizeof(frame)) {
    if (frame[6] == VISION_FRAME_TAIL) {
      vision_save_frame(tick_ms);
      frame_index = 0U;
    } else {
      frame_index = (frame[6] == VISION_FRAME_HEAD_1) ? 1U : 0U;
      if (frame_index != 0U) {
        frame[0] = VISION_FRAME_HEAD_1;
      }
    }
  }
}

void Vision_Init(void)
{
  latest_data.x = VISION_NO_TARGET_VALUE;
  latest_data.y = VISION_NO_TARGET_VALUE;
  latest_data.tick_ms = 0U;
  latest_data.valid = false;
  latest_data.stop = false;
  Vision_ResetParser();
}

void Vision_ResetParser(void)
{
  frame_index = 0U;
}

void Vision_ParseBytes(const uint8_t *data, size_t size, uint32_t tick_ms)
{
  if (data == 0) {
    return;
  }
  for (size_t i = 0U; i < size; ++i) {
    vision_parse_byte(data[i], tick_ms);
  }
}

VisionData Vision_GetSnapshot(void)
{
  VisionData snapshot;
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  snapshot.x = latest_data.x;
  snapshot.y = latest_data.y;
  snapshot.tick_ms = latest_data.tick_ms;
  snapshot.valid = latest_data.valid;
  snapshot.stop = latest_data.stop;
  if (primask == 0U) {
    __enable_irq();
  }
  return snapshot;
}

bool Vision_IsFresh(const VisionData *data, uint32_t now_ms, uint32_t timeout_ms)
{
  return (data != 0) && data->valid &&
         ((uint32_t)(now_ms - data->tick_ms) <= timeout_ms);
}
