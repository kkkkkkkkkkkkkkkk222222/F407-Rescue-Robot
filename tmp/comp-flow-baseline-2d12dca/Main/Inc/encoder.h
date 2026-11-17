#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
  int64_t position;
  int32_t delta_10ms;
} EncoderStatus;

void Encoder_Init(void);
void Encoder_Sample10ms(void);
void Encoder_GetAll(EncoderStatus status[3]);

#endif
