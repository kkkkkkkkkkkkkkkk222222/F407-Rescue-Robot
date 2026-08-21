#include "pid.h"

static float pid_limit(float value, float minimum, float maximum)
{
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}

void Pid_Init(Pid_t *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_min, float integral_max)
{
  if (pid == 0) {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->output_min = output_min;
  pid->output_max = output_max;
  pid->integral_min = integral_min;
  pid->integral_max = integral_max;
  Pid_Reset(pid);
}

void Pid_Reset(Pid_t *pid)
{
  if (pid == 0) {
    return;
  }
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
  pid->started = 0U;
}

float Pid_Update(Pid_t *pid, float target, float actual)
{
  float derivative = 0.0f;
  float output;
  const float error = target - actual;

  if (pid == 0) {
    return 0.0f;
  }

  pid->integral = pid_limit(pid->integral + error,
                            pid->integral_min,
                            pid->integral_max);
  if (pid->started != 0U) {
    derivative = error - pid->last_error;
  }
  pid->last_error = error;
  pid->started = 1U;

  output = pid->kp * error +
           pid->ki * pid->integral +
           pid->kd * derivative;
  return pid_limit(output, pid->output_min, pid->output_max);
}
