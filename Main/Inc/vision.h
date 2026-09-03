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

#define VISION_MSG_CONFIG         0x11U
#define VISION_MSG_REPORT         0x12U
#define VISION_MSG_ODOM           0x15U
#define VISION_MSG_FUSED_POSE     0x16U
#define VISION_MSG_STM_STATUS     0x17U
#define VISION_MSG_MISSION        0x18U

#define VISION_COLOR_RED          0x11U
#define VISION_COLOR_BLUE         0x12U
#define VISION_ZONE_1             0x01U
#define VISION_ZONE_2             0x02U
#define VISION_ZONE_3             0x03U
#define VISION_ZONE_4             0x04U

#define VISION_COUNT_NORMAL(v)    ((uint8_t)((v) & 0x03U))
#define VISION_COUNT_CORE(v)      ((uint8_t)(((v) >> 2) & 0x03U))
#define VISION_COUNT_CASUALTY(v)  ((uint8_t)(((v) >> 4) & 0x03U))
#define VISION_COUNT_DANGER(v)    ((uint8_t)(((v) >> 6) & 0x03U))

#define VISION_REPORT_FOUND          0x01U
#define VISION_REPORT_NEAR           0x02U
#define VISION_REPORT_CLASS_VALID    0x08U
#define VISION_REPORT_DISTANCE_VALID 0x40U

#define VISION_POSE_VALID                 0x01U
#define VISION_POSE_T265_GOOD             0x02U
#define VISION_POSE_WHEEL_ACTIVE          0x04U
#define VISION_POSE_OBSTACLE_GATE         0x08U
#define VISION_POSE_ODOM_FRESH            0x10U
#define VISION_POSE_INSIDE_FIELD          0x20U
#define VISION_POSE_T265_UPDATE_REJECTED  0x40U

#define VISION_STM_CLAW_VISIBLE    0x01U
#define VISION_STM_GRIPPER_CLOSED  0x02U
#define VISION_STM_MOTORS_ACTIVE   0x04U
#define VISION_STM_AUTO_APPROACH   0x08U
#define VISION_STM_FAULT           0x80U

#define VISION_CMD_VALID             0x01U
#define VISION_CMD_DRIVE_STRAIGHT    0x02U
#define VISION_CMD_USE_FINAL_HEADING 0x04U
#define VISION_CMD_RED_SIDE          0x08U

typedef enum {
  VISION_CMD_STOP = 0,
  VISION_CMD_GRAB_CONFIRMED = 2,
  VISION_CMD_NAVIGATE_WAYPOINT = 3,
  VISION_CMD_ALIGN_SAFE_ZONE = 4,
  VISION_CMD_ENTER_SAFE_ZONE = 5,
  VISION_CMD_TASK_COMPLETE = 6,
  VISION_CMD_ABORT = 7
} VisionMissionCode;

typedef struct {
  int16_t x_mm;
  int16_t y_mm;
  uint16_t heading_cdeg;
  uint32_t tick_ms;
  uint8_t sequence;
  uint8_t status;
  uint8_t tracker_confidence;
  uint8_t mapper_confidence;
  uint8_t position_sigma_cm;
  bool received;
} VisionFusedPose;

typedef struct {
  int16_t target_x_mm;
  int16_t target_y_mm;
  uint16_t heading_cdeg;
  uint32_t tick_ms;
  uint8_t sequence;
  uint8_t command;
  uint8_t flags;
  bool received;
} VisionMissionCommand;

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t distance_mm;
  uint32_t tick_ms;
  uint32_t last_frame_tick_ms;
  /* Local monotonically increasing ID for each accepted vision report. */
  uint32_t report_generation;
  uint8_t color;
  uint8_t start_zone;
  uint8_t config_sequence;
  uint8_t sequence;
  uint8_t cargo_counts;
  uint8_t last_frame[VISION_FRAME_SIZE];
  bool valid;
  bool frame_received;
  bool config_ready;
  bool found;
  bool near;
  bool classification_valid;
  bool distance_valid;
  VisionFusedPose fused_pose;
  VisionMissionCommand mission;
} VisionData;

typedef struct {
  uint16_t camera_pitch_cdeg;
  uint8_t flags;
  uint8_t mode;
  uint8_t acknowledged_sequence;
  uint8_t fault_code;
} VisionStmStatus;

typedef struct {
  uint16_t position[3];
  uint8_t sample_period_ms;
  uint8_t status;
} VisionOdom;

void Vision_Init(void);
void Vision_ResetParser(void);
void Vision_ParseBytes(const uint8_t *data, size_t size, uint32_t tick_ms);
VisionData Vision_GetSnapshot(void);
bool Vision_IsFresh(const VisionData *data, uint32_t now_ms,
                    uint32_t timeout_ms);
bool Vision_ReportIsFresh(const VisionData *data, uint32_t now_ms,
                          uint32_t timeout_ms);
bool Vision_FusedPoseIsFresh(const VisionFusedPose *pose, uint32_t now_ms,
                             uint32_t timeout_ms);
bool Vision_MissionIsFresh(const VisionMissionCommand *command,
                           uint32_t now_ms, uint32_t timeout_ms);
void Vision_RequestConfigAck(void);
void Vision_QueueStmStatus(const VisionStmStatus *status);
void Vision_QueueOdom(const VisionOdom *odom);
void Vision_Process(void);

#endif
