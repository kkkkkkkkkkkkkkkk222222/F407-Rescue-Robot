#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float output_min;
  float output_max;
  float integral_min;
  float integral_max;
  uint8_t started;
} Pid_t;

void Pid_Init(Pid_t *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_min, float integral_max);
void Pid_Reset(Pid_t *pid);
float Pid_Update(Pid_t *pid, float target, float actual);

#endif
