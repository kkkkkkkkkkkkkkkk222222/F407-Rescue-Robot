#ifndef LOCATION_H
#define LOCATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  LOCATION_START_1 = 1,
  LOCATION_START_2,
  LOCATION_START_3,
  LOCATION_START_4
} LocationStart;

typedef struct {
  int32_t x_mm;
  int32_t y_mm;
  int32_t heading_mdeg;
  uint32_t path_mm;
  uint8_t start_zone;
  bool valid;
  bool inside_field;
} LocationPose;

void Location_Init(LocationStart start);
void Location_Reset(LocationStart start);
void Location_Update10ms(void);
LocationPose Location_GetPose(void);

#endif
