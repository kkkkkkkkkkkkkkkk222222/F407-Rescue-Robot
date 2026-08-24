#ifndef VISION_H
#define VISION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VISION_FRAME_HEAD_1       0xA3U
#define VISION_FRAME_HEAD_2       0xB3U
#define VISION_FRAME_TAIL         0xC3U
#define VISION_FRAME_SIZE         15U
#define VISION_PAYLOAD_SIZE       8U

/* High nibble is protocol version 1; low nibble is the message type. */
#define VISION_MSG_CONFIG         0x11U
#define VISION_MSG_REPORT         0x12U
#define VISION_MSG_EVENT          0x13U
#define VISION_MSG_NAV            0x14U

#define VISION_EVENT_STOP         0x01U

#define VISION_DEST_MATERIAL      0x01U
#define VISION_DEST_CASUALTY      0x02U

#define VISION_NAV_HOLD           0x00U
#define VISION_NAV_FORWARD        0x01U
#define VISION_NAV_TURN_LEFT      0x02U
#define VISION_NAV_TURN_RIGHT     0x03U
#define VISION_NAV_BACKWARD       0x04U

#define VISION_NAV_EN_ROUTE       0x01U
#define VISION_NAV_NEAR_SAFE      0x02U

#define VISION_COLOR_RED          0x11U
#define VISION_COLOR_BLUE         0x12U

#define VISION_ZONE_1             0x01U
#define VISION_ZONE_2             0x02U
#define VISION_ZONE_3             0x03U
#define VISION_ZONE_4             0x04U

/* Vision report payload byte 6: four two-bit counts. */
#define VISION_COUNT_NORMAL(v)    ((uint8_t)((v) & 0x03U))
#define VISION_COUNT_CORE(v)      ((uint8_t)(((v) >> 2) & 0x03U))
#define VISION_COUNT_CASUALTY(v)  ((uint8_t)(((v) >> 4) & 0x03U))
#define VISION_COUNT_DANGER(v)    ((uint8_t)(((v) >> 6) & 0x03U))

/* Vision report payload byte 7. */
#define VISION_REPORT_FOUND       0x01U
#define VISION_REPORT_NEAR        0x02U
#define VISION_REPORT_GRABBED     0x04U
#define VISION_REPORT_CLASS_VALID 0x08U
#define VISION_REPORT_UNKNOWN     0x10U
#define VISION_REPORT_CLAW_VIEW   0x20U

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t distance_mm;
  uint32_t tick_ms;
  uint32_t nav_tick_ms;
  uint8_t color;
  uint8_t start_zone;
  uint8_t sequence;
  uint8_t cargo_counts;
  uint8_t nav_sequence;
  uint8_t nav_direction;
  uint8_t nav_zone_state;
  uint8_t nav_destination;
  uint8_t last_frame[VISION_FRAME_SIZE];
  bool valid;
  bool nav_valid;
  bool stop;
  bool config_ready;
  bool found;
  bool grabbed;
  bool near;
  bool classification_valid;
  bool unknown;
  bool claw_view;
} VisionData;

void Vision_Init(void);
void Vision_ResetParser(void);
void Vision_ParseBytes(const uint8_t *data, size_t size, uint32_t tick_ms);
VisionData Vision_GetSnapshot(void);
bool Vision_IsFresh(const VisionData *data, uint32_t now_ms, uint32_t timeout_ms);
bool Vision_NavIsFresh(const VisionData *data,
                       uint32_t now_ms,
                       uint32_t timeout_ms);
void Vision_RequestConfigAck(void);
void Vision_Process(void);
void Vision_OnTxComplete(void);
void Vision_OnTxError(void);

#endif
