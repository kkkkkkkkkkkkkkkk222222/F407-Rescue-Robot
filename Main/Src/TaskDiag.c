#include "TaskDiag.h"
#if APP_BOARD_F405_ROBO && APP_ENABLE_TASK
#include "main.h"
#include "Task.h"
#include "Robot.h"
#include "DebugConsole.h"
#include "vision.h"
#include "Uart.h"
#include "imu.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include <stdio.h>
#include <string.h>

#define TX_CAP 4096U
#define EVENT_CAP 64U
#define HISTORY_CAP 32U
#define LINE_CAP 512U
typedef struct { const char *kind; uint32_t ms, v[6]; } Event;
static Event events[EVENT_CAP], last_events[16];
static volatile uint16_t event_head, event_tail;
static volatile uint32_t event_drops, motor_ticks, task_ticks;
static uint32_t main_ticks, tx_drops, tx_timeouts, last_report, previous_key, record_id;
static uint8_t tx[TX_CAP], chunk[128];
static uint16_t head, tail;
static volatile bool tx_active;
static uint32_t tx_started;
static bool enabled = true, force_report = true, boot_reported;
static char history[HISTORY_CAP][LINE_CAP];
static uint8_t history_head, history_count, dump_pos, dump_left;

static uint32_t lock(void) { uint32_t m=__get_PRIMASK(); __disable_irq(); return m; }
static void unlock(uint32_t m) { __set_PRIMASK(m); }
static uint16_t free_bytes(void) { return (uint16_t)((tail + TX_CAP - head - 1U) % TX_CAP); }
static void enqueue(const char *s, bool remember)
{
  const size_t n = strlen(s);
  const uint32_t m = lock();
  if (n > free_bytes()) { ++tx_drops; unlock(m); return; }
  for (size_t i=0; i<n; ++i) { tx[head]=s[i]; head=(uint16_t)((head+1U)%TX_CAP); }
  unlock(m);
  /* Text producers run in main; task/IRQ producers only enqueue small events. */
  if (remember && dump_left == 0U && n < LINE_CAP) {
    memcpy(history[history_head], s, n+1U);
    history_head=(uint8_t)((history_head+1U)%HISTORY_CAP);
    if (history_count < HISTORY_CAP) ++history_count;
  }
}
void TaskDiag_Send(const char *text) { enqueue(text, true); }
void TaskDiag_TxComplete(void) { tx_active=false; }
void TaskDiag_MotorTick(void) { ++motor_ticks; }
void TaskDiag_TaskTick(void) { ++task_ticks; }
void TaskDiag_Enable(uint8_t value) { enabled=value != 0U; force_report=enabled; }
void TaskDiag_RequestStatus(void) { force_report=true; }
void TaskDiag_Dump(void)
{
  dump_pos=(uint8_t)((history_head+HISTORY_CAP-history_count)%HISTORY_CAP);
  dump_left=history_count;
  enqueue("TDUMP BEGIN; CACHED LINES KEEP ORIGINAL TIMESTAMPS\r\n", false);
}
void TaskDiag_Event(const char *kind, uint32_t a,uint32_t b,uint32_t c,uint32_t d,uint32_t e,uint32_t f)
{
  const Event v={kind, HAL_GetTick(), {a,b,c,d,e,f}};
  const uint32_t m=lock();
  Event *last=&last_events[0];
  for(uint8_t i=0;i<16U;++i) {
    if(last_events[i].kind==kind || last_events[i].kind==NULL) { last=&last_events[i]; break; }
  }
  /* Coalesce unchanged heartbeat decisions, never a changed primary reason. */
  if (last->kind == kind && last->v[0]==a && last->v[1]==b &&
      last->v[2]==c && v.ms-last->ms < 1000U) { unlock(m); return; }
  *last=v;
  const uint16_t next=(uint16_t)((event_head+1U)%EVENT_CAP);
  if (next==event_tail) ++event_drops;
  else { events[event_head]=v; event_head=next; }
  unlock(m);
}
static void pump(void)
{
  if (tx_active && HAL_GetTick()-tx_started > 250U) {
    const uint32_t m=lock();
    if (tx_active) {
      (void)HAL_UART_AbortTransmit(&huart1);
      tx_active=false; tail=head; ++tx_timeouts;
    }
    unlock(m);
  }
  if (tx_active || huart1.gState != HAL_UART_STATE_READY || head==tail) return;
  uint16_t cursor=tail, n=0;
  while(cursor!=head && n<sizeof(chunk)) { chunk[n++]=tx[cursor]; cursor=(uint16_t)((cursor+1U)%TX_CAP); }
  const uint32_t m=lock();
  tx_started=HAL_GetTick(); tx_active=true;
  if(HAL_UART_Transmit_IT(&huart1,chunk,n)==HAL_OK) tail=cursor;
  else tx_active=false;
  unlock(m);
}
static uint32_t wait_mask(const TaskDiagnostic *d, const TaskStatus *s, const VisionData *v)
{
  uint32_t w=0;
  if(!v->config_ready) w|=1U;
  if(d->paused) w|=2U;
  if(DebugConsole_IsActive()) w|=4U;
  if(d->command_age_ms > APP_COMMAND_RX_TIMEOUT_MS) w|=8U;
  if(d->visual_age_ms > APP_VISUAL_UPDATE_TIMEOUT_MS) w|=16U;
  if(d->audit_state) {
    if(s->camera_angle != APP_GRAB_VIEW_ANGLE) w|=32U;
    if(!s->claw_visible) w|=64U;
    const bool sweep=s->state==TASK_SAFE_SWEEP_AUDIT;
    if((sweep?d->sweep_samples:d->audit_samples)<APP_AUDIT_REQUIRED_SAMPLES) w|=128U;
    if(!(sweep?d->sweep_stable:d->audit_stable)) w|=256U;
    if(!s->audit_valid) w|=512U;
    if(s->state==TASK_TRANSPORT_AUDIT && !d->transport_camera_ready) w|=1024U;
  }
  return w;
}
static void report(const TaskDiagnostic *d,const TaskStatus *s,const VisionData *v,uint32_t wait)
{
  char b[LINE_CAP];
  const uint32_t id=++record_id;
  const IMUData imu=IMU_GetData();
  const UartLinkStatus link=Uart_GetLinkStatus();
  const VisionDiagnostic wire=Vision_GetDiagnostic();
  EncoderStatus enc[3]; Encoder_GetAll(enc);
  MotorStatus m[3]; for(uint8_t i=0;i<3;++i)m[i]=Motor_GetStatus(i+1U);
#define EMIT(...) do { (void)snprintf(b,sizeof(b),__VA_ARGS__); TaskDiag_Send(b); } while(0)
  EMIT("TD_TASK ID=%lu HAL=%lu APP=%lu INIT=%u MODE=%u FAULT=%u DWELL=%lu STEP_AGE=%lu WAIT=0x%lX TASK=%u ACTION=%u CMD=%u CTX=%u PAUSE=%u LOCAL_DEBUG=%u MAIN=%lu TASK_TICKS=%lu MOTOR_TICKS=%lu\r\n",
    (unsigned long)id,(unsigned long)HAL_GetTick(),(unsigned long)d->now_ms,d->initialized,s->state,s->fault,
    (unsigned long)d->state_age_ms,(unsigned long)d->step_age_ms,(unsigned long)wait,d->task_id,d->action_id,
    d->command,d->context_valid,d->paused,DebugConsole_IsActive(),(unsigned long)main_ticks,
    (unsigned long)task_ticks,(unsigned long)motor_ticks);
  EMIT("TD_PHASE ID=%lu SEARCH=%u APPROACH=%u POST_AUDIT=%u TRANSPORT=%u INJURY=%u REMOTE=%u/%u DONE=%u SEP=%u ATTEMPTS=%u BUDGET_MM=%lu/%u SWEEP=%u/%u FOUND=%u NAV_STALE=%u NAV_DONE=%u PUSH=%u REMAIN_S=%u\r\n",
    (unsigned long)id,d->search_phase,d->approach_phase,d->post_audit_phase,d->transport_phase,d->injury_phase,
    s->action_command,s->action_phase,s->action_done,d->separation_stage,d->separation_attempts,
    (unsigned long)d->capture_used_mm,APP_CAPTURE_FORWARD_BUDGET_MM,s->sweep_phase,s->sweep_reverse_phase,
    s->found,s->nav_stale,s->nav_done,s->nav_final_push,s->remaining_s);
  EMIT("TD_AUDIT ID=%lu FRAME=%lu AID=%u N=%u/%u STABLE=%u READY=%u VALID=%u SEM=%u AGE=%lu SAMPLE_AGE=%lu MIN_MS=%u MAX_GAP_MS=%u CLASS=%u,%u TOTAL=%u FLAGS=0x%X CLAW_VISIBLE=%u CLOSED=%u CAM_TARGET=%u REQUIRED=%u CAM_WAIT_OK=%u SWEEP=%u,%u,%u,%u\r\n",
    (unsigned long)id,(unsigned long)d->frame,d->audit_id,d->audit_samples,APP_AUDIT_REQUIRED_SAMPLES,
    d->audit_stable,s->audit_ready,s->audit_valid,d->audit_semantic,(unsigned long)d->audit_age_ms,
    (unsigned long)d->sample_age_ms,APP_AUDIT_SAMPLE_INTERVAL_MS,APP_AUDIT_EVIDENCE_MAX_GAP_MS,
    d->audit_left,d->audit_right,d->audit_total,d->audit_flags,s->claw_visible,s->gripper_closed,
    s->camera_angle,APP_GRAB_VIEW_ANGLE,d->transport_camera_ready,d->sweep_samples,d->sweep_stable,d->sweep_ready,d->sweep_valid);
  EMIT("TD_LINK ID=%lu CONFIG=%u CMD_AGE=%lu/%u VIS_AGE=%lu/%u TARGET=%d,%d SEEN=%lu ACCEPT=%lu REJECT=%lu LAST_REJECT_STAGE=%u REJECT_ACTION=%u RXERR=%lu RXCODE=0x%lX RX_RESTART=%lu RX_ACTIVE=%u TX_DROP=%lu TX_LEFT=%u POSE_RX=%u POSE_AGE=%lu POSE_FLAGS=0x%X\r\n",
    (unsigned long)id,v->config_ready,(unsigned long)d->command_age_ms,APP_COMMAND_RX_TIMEOUT_MS,
    (unsigned long)d->visual_age_ms,APP_VISUAL_UPDATE_TIMEOUT_MS,d->target_x,d->target_y,
    (unsigned long)d->rx_count,(unsigned long)d->accept_count,(unsigned long)d->reject_count,d->reject_stage,
    d->rejected_action_id,(unsigned long)link.rx_errors,(unsigned long)link.last_rx_error,(unsigned long)link.rx_restarts,
    link.rx_active,(unsigned long)link.tx_dropped,link.tx_remaining,v->fused_pose.received,
    (unsigned long)(v->fused_pose.received?d->now_ms-v->fused_pose.tick_ms:UINT32_MAX),v->fused_pose.status);
  EMIT("TD_DRIVE ID=%lu TARGET=%d,%d,%d ACTUAL=%d,%d,%d PWM=%d,%d,%d RAW32=%ld,%ld,%ld SERVO_TARGET=%u,%u,%u,%u IMU=%u UNCERTAIN=%u AGE=%lu YAW_MOD=%ld RATE=%ld UERR=%lu LOG_DROP=%lu EVT_DROP=%lu TX_TIMEOUT=%lu\r\n",
    (unsigned long)id,m[0].target_speed_mm_s,m[1].target_speed_mm_s,m[2].target_speed_mm_s,
    m[0].measured_speed_mm_s,m[1].measured_speed_mm_s,m[2].measured_speed_mm_s,m[0].command,m[1].command,m[2].command,
    (long)(int32_t)enc[0].position,(long)(int32_t)enc[1].position,(long)(int32_t)enc[2].position,
    Servo_GetAngle(1),Servo_GetAngle(2),Servo_GetAngle(3),Servo_GetAngle(4),imu.ready,imu.heading_uncertain,
    (unsigned long)imu.uart_angle_age_ms,(long)(imu.yaw_mdeg%360000LL),(long)imu.gyro_mdps[2],
    (unsigned long)imu.uart_rx_errors,(unsigned long)tx_drops,(unsigned long)event_drops,(unsigned long)tx_timeouts);
  EMIT("TD_WIRE ID=%lu BYTES=%lu VALID_FRAMES=%lu INVALID_FRAMES=%lu RX_AGE=%lu VALID_AGE=%lu TYPE=0x%X SEQ=%u STATUS_QUEUED=%lu STATUS_IS_NOT_HOST_ACK=1\r\n",
    (unsigned long)id,(unsigned long)wire.rx_bytes,(unsigned long)wire.valid_frames,
    (unsigned long)wire.invalid_frames,(unsigned long)(wire.rx_bytes?d->now_ms-wire.last_rx_ms:UINT32_MAX),
    (unsigned long)(wire.valid_frames?d->now_ms-wire.last_valid_ms:UINT32_MAX),
    wire.last_type,wire.last_sequence,(unsigned long)wire.status_queued);
#undef EMIT
}
void TaskDiag_Process(void)
{
  ++main_ticks;
  pump();
  if(!boot_reported) {
    char b[LINE_CAP];
    (void)snprintf(b,sizeof(b),"TD_BOOT FORMAT=1 BOARD=F405 TASK=NormalRun RESET=0x%08lX BUILT=" __DATE__ " " __TIME__ "\r\n",(unsigned long)RCC->CSR);
    TaskDiag_Send(b); boot_reported=true;
  }
  if(dump_left && free_bytes() >= LINE_CAP) {
    enqueue(history[dump_pos],false); dump_pos=(uint8_t)((dump_pos+1U)%HISTORY_CAP);
    if(--dump_left==0U)enqueue("TDUMP END\r\n",false);
  }
  for(uint8_t i=0;i<4U && free_bytes()>=LINE_CAP;++i) {
    Event e;
    const uint32_t m=lock();
    if(event_tail==event_head) { unlock(m); break; }
    e=events[event_tail]; event_tail=(uint16_t)((event_tail+1U)%EVENT_CAP); unlock(m);
    char b[LINE_CAP];
    (void)snprintf(b,sizeof(b),"TD_EVENT HAL=%lu KIND=%s A=%lu B=%lu C=%lu D=%lu E=%lu F=%lu\r\n",
      (unsigned long)e.ms,e.kind,(unsigned long)e.v[0],(unsigned long)e.v[1],(unsigned long)e.v[2],
      (unsigned long)e.v[3],(unsigned long)e.v[4],(unsigned long)e.v[5]);
    if(enabled)TaskDiag_Send(b);
  }
  const uint32_t now=Robot_GetMilliseconds();
  const TaskDiagnostic d=Task_GetDiagnostic(now);
  const TaskStatus s=Task_GetStatus();
  const VisionData v=Vision_GetSnapshot();
  const uint32_t w=wait_mask(&d,&s,&v);
  const uint32_t key=(uint32_t)s.state ^ (w<<8U) ^ ((uint32_t)d.approach_phase<<24U) ^
      ((uint32_t)s.action_phase<<20U) ^ ((uint32_t)d.search_phase<<28U);
  if(free_bytes()>=6U*LINE_CAP && (force_report || (enabled &&
      (now-last_report>=1000U || (key!=previous_key && now-last_report>=100U))))) {
    report(&d,&s,&v,w); last_report=now; previous_key=key; force_report=false;
  }
  pump();
}
#endif
