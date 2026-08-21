#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
void Encoder_Sample10ms(void);
int32_t Encoder_Get(uint8_t id);
int32_t Encoder_GetDelta10ms(uint8_t id);
int32_t Encoder_TakeControlDelta(uint8_t id);
void Encoder_Reset(uint8_t id);
void Encoder_OnExti(uint16_t gpio_pin);

#endif
