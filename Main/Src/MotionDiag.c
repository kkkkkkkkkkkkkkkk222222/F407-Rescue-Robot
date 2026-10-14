#include "MotionDiag.h"

#if APP_ENABLE_MOTION_DEBUG_TASK
#include <limits.h>
#include <stdio.h>
#include "Debug.h"
#include "imu.h"
#include "main.h"
#include "motor.h"

#define DIAG_MAGIC 0x4D444731UL
#define DIAG_FORMAT 3U

typedef struct {
  uint32_t valid, exception, pc, lr, xpsr, stack_pointer, exc_return;
  uint32_t cfsr, hfsr, dfsr, afsr, bfar, mmfar, icsr, primask, basepri, faultmask;
  uint32_t frame_valid;
} MotionFault;

typedef struct {
  uint32_t magic, inverse_magic, format, size;
  uint32_t have_sample, app_ms, hal_ms, main_loops, task_loops, motor_ticks;
  uint32_t stage, main_ms, max_main_gap_ms, last_imu_ms;
  uint32_t command, sequence, source, motion_state;
  MotorDebugTrace motor;
  IMUData imu;
  MotionFault fault;
} MotionRecord;

static volatile MotionRecord live_record __attribute__((section(".noinit.motion_diag"), aligned(8)));
static MotionRecord previous_record;
static uint32_t reset_flags;
static bool previous_valid;
uint32_t motion_diag_fault_stack[128] __attribute__((aligned(8), used));

static long bounded_i64(int64_t value)
{
  return value > INT32_MAX ? INT32_MAX : value < INT32_MIN ? INT32_MIN : (long)value;
}

void MotionDiag_Init(void)
{
  reset_flags = RCC->CSR;
  previous_valid = (reset_flags & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)) == 0U &&
      live_record.magic == DIAG_MAGIC && live_record.inverse_magic == ~DIAG_MAGIC &&
      live_record.format == DIAG_FORMAT && live_record.size == sizeof(MotionRecord);
  if (previous_valid) { previous_record = live_record; }
  live_record = (MotionRecord){.magic = DIAG_MAGIC, .inverse_magic = ~DIAG_MAGIC,
                              .format = DIAG_FORMAT, .size = sizeof(MotionRecord)};
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

void MotionDiag_SetStage(MotionDiagStage stage, uint32_t app_ms)
{
  live_record.stage = stage;
  if (stage == MOTION_DIAG_MAIN) {
    ++live_record.main_loops;
    live_record.main_ms = app_ms;
  }
}

void MotionDiag_TaskTick(void)
{
  ++live_record.task_loops;
}

void MotionDiag_MotorTick(uint32_t app_ms)
{
  live_record.app_ms = app_ms;
  live_record.hal_ms = HAL_GetTick();
  ++live_record.motor_ticks;
  const uint32_t main_gap = app_ms - live_record.main_ms;
  if (main_gap > live_record.max_main_gap_ms) { live_record.max_main_gap_ms = main_gap; }
  const IMUData imu = IMU_GetData();
  if (imu.sample_count != live_record.imu.sample_count) { live_record.last_imu_ms = app_ms; }
  live_record.imu = imu;
  live_record.motor = Motor_GetDebugTrace();
  const DebugStatus status = Debug_GetStatus();
  live_record.command = status.command;
  live_record.sequence = status.command_sequence;
  live_record.source = status.source;
  live_record.motion_state = status.state;
  live_record.have_sample = 1U;
}

static void print_record(void (*send)(const char *), const char *prefix, const MotionRecord *r)
{
  char text[384];
  const MotorDebugTrace *m = &r->motor;
  const int32_t main_age = (int32_t)(r->app_ms - r->main_ms);
  (void)snprintf(text, sizeof(text),
      "%s LOOP APP_MS=%lu HAL_MS=%lu MAIN=%lu TASK=%lu MOTOR=%lu STAGE=%lu MAIN_AGE=%lu MAX_MAIN_GAP=%lu CMD=%lu SEQ=%lu SRC=%lu STATE=%lu\r\n",
      prefix, (unsigned long)r->app_ms, (unsigned long)r->hal_ms,
      (unsigned long)r->main_loops, (unsigned long)r->task_loops, (unsigned long)r->motor_ticks,
      (unsigned long)r->stage, (unsigned long)(main_age > 0 ? main_age : 0),
      (unsigned long)r->max_main_gap_ms, (unsigned long)r->command,
      (unsigned long)r->sequence, (unsigned long)r->source, (unsigned long)r->motion_state);
  send(text);
  (void)snprintf(text, sizeof(text),
      "%s TURN TARGET_MDEG=%ld RAW_DELTA_MDEG=%ld MOTOR_STATE=%u SLOW=%u BRAKE=%u PAUSED=%u ELAPSED_MS=%lu Z_MDPS=%ld\r\n",
      prefix, (long)m->turn_target_mdeg, bounded_i64(m->turn_delta_mdeg), m->turn_state,
      m->slowing, m->brake_cycles, m->paused, (unsigned long)m->turn_elapsed_ms,
      (long)r->imu.gyro_mdps[2]);
  send(text);
  (void)snprintf(text, sizeof(text),
      "%s TURN_CTL PHASE=%u ERR_MDEG=%ld REF_MDPS=%ld CMD_MDPS=%ld SETTLE_MS=%u TRIMS=%u FAIL=%u IMU_AGE_MS=%lu REST_MS=%u BRAKE_PRED_MDEG=%ld BOOST=%u BOOST_MS=%u BOOSTS=%u BOOST_EXIT=%u\r\n",
      prefix, m->turn_phase, (long)m->turn_error_mdeg, (long)m->turn_reference_mdps,
      (long)m->turn_command_mdps, m->turn_settle_ms, m->turn_corrections,
      m->turn_failure, (unsigned long)m->turn_imu_age_ms, m->turn_rest_ms,
      (long)m->turn_brake_distance_mdeg, m->turn_boost_active, m->turn_boost_ms,
      m->turn_boost_count, m->turn_boost_exit);
  send(text);
  (void)snprintf(text, sizeof(text),
      "%s PI ACTIVE=%u ERR_C100=%ld,%ld,%ld P100=%ld,%ld,%ld I100=%ld,%ld,%ld FF=%d,%d,%d CORR100=%ld,%ld,%ld PWM=%d,%d,%d\r\n",
      prefix, m->pi_active_mask, (long)m->error_c100[0], (long)m->error_c100[1], (long)m->error_c100[2],
      (long)m->p_pwm100[0], (long)m->p_pwm100[1], (long)m->p_pwm100[2],
      (long)m->i_pwm100[0], (long)m->i_pwm100[1], (long)m->i_pwm100[2],
      m->feedforward[0], m->feedforward[1], m->feedforward[2],
      (long)m->correction_pwm100[0], (long)m->correction_pwm100[1], (long)m->correction_pwm100[2],
      m->output[0], m->output[1], m->output[2]);
  send(text);
  (void)snprintf(text, sizeof(text),
      "%s DRIVE TARGET=%d,%d,%d ACTUAL=%d,%d,%d RAW_D10=%ld,%ld,%ld FB_D10=%ld,%ld,%ld\r\n",
      prefix, m->target_mm_s[0], m->target_mm_s[1], m->target_mm_s[2],
      m->actual_mm_s[0], m->actual_mm_s[1], m->actual_mm_s[2],
      (long)m->raw_count[0], (long)m->raw_count[1], (long)m->raw_count[2],
      (long)m->feedback_count[0], (long)m->feedback_count[1], (long)m->feedback_count[2]);
  send(text);
  (void)snprintf(text, sizeof(text),
      "%s IMU READY=%u SAMPLE=%lu AGE_MS=%lu YAW_MDEG=%ld DT_US=%lu MAX_DT_US=%lu GAPS=%lu ERRORS=%lu BIAS_Z_MDPS=%ld\r\n",
      prefix, r->imu.ready, (unsigned long)r->imu.sample_count,
      (unsigned long)(r->app_ms - r->last_imu_ms), bounded_i64(r->imu.yaw_mdeg),
      (unsigned long)r->imu.sample_interval_us, (unsigned long)r->imu.max_sample_interval_us,
      (unsigned long)r->imu.integration_gap_count, (unsigned long)r->imu.error_count,
      (long)r->imu.gyro_bias_mdps[2]);
  send(text);
}

void MotionDiag_Print(void (*send)(const char *), const char *prefix)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const MotionRecord record = live_record;
  if (mask == 0U) { __enable_irq(); }
  print_record(send, prefix, &record);
}

void MotionDiag_PrintPrevious(void (*send)(const char *))
{
  char text[384];
  (void)snprintf(text, sizeof(text), "PREV VALID=%u RESET_FLAGS=0x%08lX FORMAT=%u\r\n",
      previous_valid, (unsigned long)reset_flags, DIAG_FORMAT);
  send(text);
  if (!previous_valid) { return; }
  const MotionFault *f = &previous_record.fault;
  (void)snprintf(text, sizeof(text),
      "PREV FAULT=%lu EXC=%lu PC=0x%08lX LR=0x%08lX XPSR=0x%08lX SP=0x%08lX EXC_RETURN=0x%08lX FRAME_VALID=%lu\r\n",
      (unsigned long)f->valid, (unsigned long)f->exception, (unsigned long)f->pc,
      (unsigned long)f->lr, (unsigned long)f->xpsr, (unsigned long)f->stack_pointer,
      (unsigned long)f->exc_return, (unsigned long)f->frame_valid);
  send(text);
  (void)snprintf(text, sizeof(text),
      "PREV CFSR=0x%08lX HFSR=0x%08lX DFSR=0x%08lX AFSR=0x%08lX BFAR=0x%08lX MMFAR=0x%08lX ICSR=0x%08lX PRIMASK=%lu BASEPRI=%lu FAULTMASK=%lu\r\n",
      (unsigned long)f->cfsr, (unsigned long)f->hfsr, (unsigned long)f->dfsr,
      (unsigned long)f->afsr, (unsigned long)f->bfar, (unsigned long)f->mmfar,
      (unsigned long)f->icsr, (unsigned long)f->primask, (unsigned long)f->basepri,
      (unsigned long)f->faultmask);
  send(text);
  if (previous_record.have_sample == 1U) { print_record(send, "PREV", &previous_record); }
}

static void emergency_outputs_off(void)
{
  /* Bypass HAL, locks and floating-point state even if the interrupted stack
   * is corrupt. Force all six AT8236 inputs low as GPIO outputs. */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOEEN;
  __DSB();
  GPIOA->BSRR = (GPIO_PIN_2 | GPIO_PIN_3) << 16U;
  GPIOB->BSRR = (GPIO_PIN_10 | GPIO_PIN_11) << 16U;
  GPIOE->BSRR = (GPIO_PIN_5 | GPIO_PIN_6) << 16U;
  GPIOA->MODER = (GPIOA->MODER & ~0x000000F0UL) | 0x00000050UL;
  GPIOB->MODER = (GPIOB->MODER & ~0x00F00000UL) | 0x00500000UL;
  GPIOE->MODER = (GPIOE->MODER & ~0x00003C00UL) | 0x00001400UL;
}

static void fault_save(uint32_t *stack, uint32_t exc_return, uint32_t exception,
                       uint32_t caller) __attribute__((noreturn));
static void fault_save(uint32_t *stack, uint32_t exc_return, uint32_t exception,
                       uint32_t caller)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  emergency_outputs_off();
  if (live_record.magic != DIAG_MAGIC || live_record.inverse_magic != ~DIAG_MAGIC) {
    live_record.have_sample = 0U;
    live_record.fault.valid = 0U;
  }
  live_record.magic = DIAG_MAGIC;
  live_record.inverse_magic = ~DIAG_MAGIC;
  live_record.format = DIAG_FORMAT;
  live_record.size = sizeof(MotionRecord);
  if (live_record.fault.valid != 1U) {
    volatile MotionFault *f = &live_record.fault;
    f->exception = exception;
    f->stack_pointer = (uint32_t)stack;
    f->exc_return = exc_return;
    f->primask = mask;
    f->basepri = __get_BASEPRI();
    f->faultmask = __get_FAULTMASK();
    f->cfsr = SCB->CFSR;
    f->hfsr = SCB->HFSR;
    f->dfsr = SCB->DFSR;
    f->afsr = SCB->AFSR;
    f->bfar = SCB->BFAR;
    f->mmfar = SCB->MMFAR;
    f->icsr = SCB->ICSR;
    f->pc = caller; f->lr = 0U; f->xpsr = 0U; f->frame_valid = 0U;
    const uint32_t address = (uint32_t)stack;
    const uint32_t core_address = address + ((exc_return & 0x10U) ? 0U : 18U * 4U);
    const bool in_ram = core_address >= address &&
        ((address >= 0x20000000UL && core_address <= 0x20020000UL - 32U) ||
         (address >= 0x10000000UL && core_address <= 0x10010000UL - 32U));
    if (stack != NULL && in_ram && (address & 3U) == 0U &&
        (f->cfsr & 0x00003838UL) == 0U) {
      const uint32_t *core = (const uint32_t *)core_address;
      f->lr = core[5]; f->pc = core[6]; f->xpsr = core[7]; f->frame_valid = 1U;
    }
    f->valid = 1U;
  }
  __DSB();
  for (;;) { __NOP(); }
}

void MotionDiag_FaultCapture(uint32_t *stack, uint32_t exc_return)
    __attribute__((used, noreturn));
void MotionDiag_FaultCapture(uint32_t *stack, uint32_t exc_return)
{
  fault_save(stack, exc_return, __get_IPSR(), 0U);
}

void MotionDiag_FaultEntry(void)
{
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "ldr r2, =motion_diag_fault_stack + 512\n"
      "msr msp, r2\n"
      "b MotionDiag_FaultCapture\n");
}

void MotionDiag_Error(uint32_t caller)
{
  fault_save(NULL, 0U, 256U, caller);
}
#endif
