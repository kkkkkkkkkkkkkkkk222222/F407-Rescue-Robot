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

static float pid_update(Pid_t *pid, float target, float actual,
                        float integral_scale, float derivative_scale)
{
  float derivative = 0.0f;
  float candidate_integral;
  float output;
  float error;

  if (pid == 0) {
    return 0.0f;
  }
  error = target - actual;

  candidate_integral = pid_limit(pid->integral + error * integral_scale,
                                 pid->integral_min,
                                 pid->integral_max);
  if (pid->started != 0U) {
    derivative = (error - pid->last_error) * derivative_scale;
  }
  pid->last_error = error;
  pid->started = 1U;

  output = pid->kp * error +
           pid->ki * candidate_integral +
           pid->kd * derivative;

  /* Do not keep integrating when the error would push an already saturated
   * output farther into saturation. Integration resumes automatically when
   * the error helps the controller return to its usable output range. */
  if (!(((output > pid->output_max) && (error > 0.0f)) ||
        ((output < pid->output_min) && (error < 0.0f)))) {
    pid->integral = candidate_integral;
  } else {
    output = pid->kp * error +
             pid->ki * pid->integral +
             pid->kd * derivative;
  }
  return pid_limit(output, pid->output_min, pid->output_max);
}

float Pid_Update(Pid_t *pid, float target, float actual)
{
  return pid_update(pid, target, actual, 1.0f, 1.0f);
}

float Pid_UpdateDt(Pid_t *pid, float target, float actual, float dt_s)
{
  if ((pid == 0) || !(dt_s > 0.0f)) {
    return 0.0f;
  }
  return pid_update(pid, target, actual, dt_s, 1.0f / dt_s);
}
