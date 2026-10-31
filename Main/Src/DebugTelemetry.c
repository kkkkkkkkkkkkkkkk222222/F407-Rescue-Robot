#include "DebugTelemetry.h"
#if DEBUG_TIMED_LINK
#include "main.h"
#include "imu.h"
#include "Uart.h"
#include "DebugBuildIdentity.h"
#include <limits.h>
#include <math.h>
#include <string.h>

/* Extended namespace D4 4D, not a reinterpretation of the legacy A3 B3 frames. */
enum { ID_REQ=1, ID_REPLY, SYNC_REQ, SYNC_REPLY, CONFIG_REQ, CONFIG_REPLY,
       WHEEL=7, GYRO=8, STATS_REQ=9, STATS_REPLY=10, ERROR_REPLY=127 };
enum { OK=0, BAD_VERSION=1, BAD_TYPE=2, BAD_LENGTH=3, BANDWIDTH=4, BAD_PROFILE=5 };
enum { W_VALID=1, W_TIME_BOUND=2, W_PENDING=4, W_TRANSITION=8,
       W_INTERVAL=16, W_CLIPPED=32, W_GAP=64, W_SLIP_UNKNOWN=128, W_EPOCH_BAD=256 };
enum { G_VALID=1, G_READY=2, G_RECOVERING=4, G_UNCERTAIN=8, G_STALE=16,
       G_CLIPPED=32, G_NEW=64, G_REPEAT=128, G_RX_TIME=256, G_TIME_UNKNOWN=512,
       G_EPOCH_BAD=1024 };
#define MAX_PAYLOAD 160U
#define REQUEST_MAX 64U
#define REQUEST_SLOTS 4U
#define WHEEL_PERIOD_US 20000U
#define GYRO_PERIOD_US 10000U
#define REQUIRED_BAUD 230400U
static const uint8_t firmware_id[32] = DEBUG_FIRMWARE_ID_BYTES;
static const uint8_t git_id[20] = DEBUG_GIT_ID_BYTES;
static uint8_t epoch[16];
static bool epoch_valid, initialized, streaming;
static volatile uint32_t reference_epoch;
static uint16_t packet_sequence;
static volatile uint32_t stats[24];

typedef struct {
  uint64_t t0, t1;
  uint32_t id, reference, flags, capture_us;
  int32_t velocity[2], delta[3];
  uint32_t raw[3];
} WheelSample;
static volatile WheelSample latest_wheel;
static WheelSample accumulation;
static uint64_t previous_wheel_us;
static volatile uint64_t reference_change_us;
static uint64_t last_wheel_report_us;
static uint32_t previous_bad, wheel_intervals;
static uint32_t consumed_wheel;

typedef struct { uint64_t received; int64_t value; uint32_t id; } GyroSample;
static volatile GyroSample latest_yaw, latest_rate;
static uint32_t published_yaw, published_rate, gyro_id;
static uint64_t next_gyro_us;

typedef struct { uint8_t bytes[REQUEST_MAX]; uint16_t size; uint64_t received; } Request;
static Request requests[REQUEST_SLOTS];
static volatile uint8_t request_head, request_tail;
static uint8_t rx_buffer[REQUEST_MAX];
static uint16_t rx_size, rx_expected;
static uint64_t rx_last_us;

static uint32_t enter(void) { uint32_t m=__get_PRIMASK(); __disable_irq(); return m; }
static void leave(uint32_t m) { __set_PRIMASK(m); }
static uint32_t bounded_u32(uint64_t x) { return x>UINT32_MAX ? UINT32_MAX : (uint32_t)x; }
static void maximum(unsigned i, uint64_t x) { if(x>stats[i]) stats[i]=bounded_u32(x); }
static uint16_t crc16(const uint8_t *p, uint16_t n)
{
  uint16_t c=0xffffU;
  for(uint16_t i=0;i<n;++i) { c^=p[i]; for(uint8_t b=0;b<8;++b) c=(c&1U)?(c>>1)^0xa001U:c>>1; }
  return c;
}
static uint64_t read_be(const uint8_t *p, uint8_t n)
{ uint64_t v=0; for(uint8_t i=0;i<n;++i) v=(v<<8)|p[i]; return v; }
static void put(uint8_t *p, uint16_t *offset, uint64_t value, uint8_t n)
{ for(uint8_t i=n;i>0;--i) p[(*offset)++]=(uint8_t)(value>>((i-1U)*8U)); }
static void prefix(uint8_t *p, uint16_t *n)
{ memcpy(p,epoch,16); *n=16; put(p,n,reference_epoch,4); }

static bool send_frame(uint8_t type, uint16_t seq, const uint8_t *p, uint16_t n)
{
  if(n>MAX_PAYLOAD) return false;
  uint8_t frame[MAX_PAYLOAD+11U]={0xd4,0x4d,1,type,(uint8_t)(n>>8),(uint8_t)n,
                                (uint8_t)(seq>>8),(uint8_t)seq};
  memcpy(frame+8,p,n);
  const uint16_t crc=crc16(frame+2,n+6U);
  frame[n+8U]=(uint8_t)crc; frame[n+9U]=(uint8_t)(crc>>8); frame[n+10U]=0xe4;
  const bool accepted=Uart_Send(frame,n+11U);
  if(!accepted) ++stats[11];
  return accepted;
}

extern void DebugClock_Init(void);
extern bool DebugClock_NewEpoch(uint8_t epoch[16]);
void DebugTelemetry_Init(void)
{
  initialized=false; streaming=false; reference_epoch=0U; packet_sequence=0U;
  previous_wheel_us=reference_change_us=last_wheel_report_us=0U;
  previous_bad=wheel_intervals=consumed_wheel=0U;
  published_yaw=published_rate=gyro_id=0U; next_gyro_us=0U;
  request_head=request_tail=0U; rx_size=rx_expected=0U; rx_last_us=0U;
  memset(&accumulation,0,sizeof(accumulation));
  memset((void *)&latest_wheel,0,sizeof(latest_wheel));
  memset((void *)&latest_yaw,0,sizeof(latest_yaw));
  memset((void *)&latest_rate,0,sizeof(latest_rate));
  DebugClock_Init();
  memset((void *)stats,0,sizeof(stats));
  epoch_valid=DebugClock_NewEpoch(epoch);
  /* RNG failure disables qualification, never robot motion and never fabricates an epoch. */
  initialized=true;
}

void DebugTelemetry_ReferenceReset(void)
{
  if(!initialized) return;
  const uint32_t m=enter(); ++reference_epoch;
  reference_change_us=DebugTelemetry_NowUs(); leave(m);
}

void DebugTelemetry_WheelSample(uint64_t begin, uint64_t end,
    const int64_t position[3], const int32_t delta[3], uint32_t bad, bool pending)
{
  if(!initialized) return;
  const uint64_t middle=begin+(end-begin)/2U;
  const uint32_t span=bounded_u32(end-begin);
  maximum(15,span);
  if(!previous_wheel_us) { previous_wheel_us=middle; previous_bad=bad; return; }
  if(!wheel_intervals) {
    accumulation.t0=previous_wheel_us;
    accumulation.flags=W_SLIP_UNKNOWN;
    accumulation.capture_us=span;
    memset(accumulation.delta,0,sizeof(accumulation.delta));
  }
  if(pending) accumulation.flags|=W_PENDING;
  if(bad!=previous_bad) accumulation.flags|=W_TRANSITION;
  if(middle-previous_wheel_us>15000U) accumulation.flags|=W_GAP;
  for(uint8_t i=0;i<3U;++i) {
    const int64_t sum=(int64_t)accumulation.delta[i]+delta[i];
    if(sum>INT32_MAX || sum<INT32_MIN) accumulation.flags|=W_CLIPPED;
    accumulation.delta[i]=sum>INT32_MAX ? INT32_MAX : sum<INT32_MIN ? INT32_MIN : (int32_t)sum;
    accumulation.raw[i]=(uint32_t)position[i];
  }
  if(span>accumulation.capture_us) accumulation.capture_us=span;
  previous_bad=bad; previous_wheel_us=middle;
  if(++wheel_intervals<2U) return;
  wheel_intervals=0U;
  accumulation.t1=middle;
  const uint64_t duration=middle-accumulation.t0;
  maximum(20,duration);
  if(!duration || duration>50000U) accumulation.flags|=W_INTERVAL;
  if(!epoch_valid) accumulation.flags|=W_EPOCH_BAD;
  if(!(accumulation.flags & ~(uint32_t)W_SLIP_UNKNOWN)) {
    const float scale=(3.14159265358979323846f*APP_WHEEL_DIAMETER_MM/
        APP_ENCODER_COUNTS_PER_WHEEL_REV)*1000000000.0f/(float)duration;
    const float vx=scale*((float)accumulation.delta[0]-accumulation.delta[1])/1.7320508075688772f;
    const float vy=scale*((float)accumulation.delta[0]+accumulation.delta[1]-2.0f*accumulation.delta[2])/3.0f;
    if(!isfinite(vx) || !isfinite(vy) || fabsf(vx)>=2147483647.0f || fabsf(vy)>=2147483647.0f) {
      accumulation.flags|=W_CLIPPED;
    } else {
      accumulation.velocity[0]=(int32_t)roundf(vx);
      accumulation.velocity[1]=(int32_t)roundf(vy);
      accumulation.flags|=W_VALID;
    }
  }
  accumulation.reference=reference_epoch;
  if(accumulation.t0<reference_change_us && reference_change_us<=accumulation.t1) {
    accumulation.flags=(accumulation.flags & ~(uint32_t)W_VALID)|1024U;
    accumulation.velocity[0]=accumulation.velocity[1]=INT32_MIN;
  }
  if(!(accumulation.flags&W_VALID)) {
    accumulation.velocity[0]=accumulation.velocity[1]=INT32_MIN; ++stats[6];
  }
  accumulation.id=++stats[5];
  latest_wheel=accumulation;
}

void DebugTelemetry_GyroSample(uint8_t type, int64_t yaw, int32_t rate, uint64_t received)
{
  if(!initialized) return;
  volatile GyroSample *s=type==0xbbU ? &latest_yaw : &latest_rate;
  if(s->received) maximum(type==0xbbU ? 18U:19U,received-s->received);
  s->received=received; s->value=type==0xbbU ? yaw : rate; ++s->id;
}

bool DebugTelemetry_RxByte(uint8_t value, bool legacy_idle)
{
  if(!initialized) return false;
  if(!rx_size && (!legacy_idle || value!=0xd4U)) return false;
  const uint64_t now=DebugTelemetry_NowUs();
  if(rx_size && now-rx_last_us>20000U) { rx_size=0U; ++stats[4]; }
  rx_last_us=now;
  if(!rx_size) { if(value!=0xd4U) return false; rx_buffer[rx_size++]=value; return true; }
  if(rx_size==1U && value!=0x4dU) {
    rx_size=value==0xd4U ? 1U:0U; return value==0xd4U;
  }
  rx_buffer[rx_size++]=value;
  if(rx_size==8U) {
    rx_expected=(uint16_t)read_be(rx_buffer+4,2)+11U;
    if(rx_expected>REQUEST_MAX || rx_expected<11U) { rx_size=0; ++stats[2]; return true; }
  }
  if(rx_size>=8U && rx_size==rx_expected) {
    const uint8_t next=(uint8_t)((request_head+1U)%REQUEST_SLOTS);
    if(next==request_tail) ++stats[3];
    else {
      Request *r=&requests[request_head];
      memcpy(r->bytes,rx_buffer,rx_size); r->size=rx_size; r->received=now;
      __DMB(); request_head=next;
    }
    rx_size=0U;
  }
  return true;
}

static uint32_t gyro_flags(const GyroSample *s, const IMUData *imu, uint64_t now,
                           uint32_t previous_id, bool yaw)
{
  uint32_t f=G_RX_TIME|G_TIME_UNKNOWN;
  if(imu->ready) f|=G_READY;
  if(imu->recovering) f|=G_RECOVERING;
  if(yaw && imu->heading_uncertain) f|=G_UNCERTAIN;
  if(!yaw && (s->value>=1999900LL || s->value<=-1999900LL)) f|=G_CLIPPED;
  if(!s->received || now-s->received>20000U) f|=G_STALE;
  f|=s->received && s->id!=previous_id ? G_NEW:G_REPEAT;
  if(!epoch_valid) f|=G_EPOCH_BAD;
  if((f&G_READY) && !(f&(G_RECOVERING|G_UNCERTAIN|G_STALE|G_CLIPPED|G_EPOCH_BAD))) f|=G_VALID;
  return f;
}

static void publish_gyro(uint64_t now)
{
  GyroSample yaw,rate;
  const uint32_t m=enter(); yaw=latest_yaw; rate=latest_rate; leave(m);
  const IMUData imu=IMU_GetData();
  uint8_t p[MAX_PAYLOAD]; uint16_t n;
  prefix(p,&n); put(p,&n,++gyro_id,4);
  put(p,&n,(uint64_t)yaw.value,8); put(p,&n,yaw.received,8); put(p,&n,yaw.id,4);
  put(p,&n,gyro_flags(&yaw,&imu,now,published_yaw,true),4); put(p,&n,UINT32_MAX,4);
  put(p,&n,(uint32_t)rate.value,4); put(p,&n,rate.received,8); put(p,&n,rate.id,4);
  put(p,&n,gyro_flags(&rate,&imu,now,published_rate,false),4); put(p,&n,UINT32_MAX,4);
  put(p,&n,now,8);
  if(send_frame(GYRO,packet_sequence++,p,n)) {
    ++stats[8];
    if(yaw.id==published_yaw || rate.id==published_rate) ++stats[9];
    published_yaw=yaw.id; published_rate=rate.id;
  }
}

static void publish_wheel(void)
{
  WheelSample s; const uint32_t m=enter(); s=latest_wheel; leave(m);
  const uint64_t now=DebugTelemetry_NowUs();
  if(!s.t1 || s.id==consumed_wheel) {
    if(now-last_wheel_report_us<WHEEL_PERIOD_US || (s.t1 && now-s.t1<30000U)) return;
    s.flags=(s.flags & ~(uint32_t)W_VALID)|512U;
    s.velocity[0]=s.velocity[1]=INT32_MIN;
  }
  last_wheel_report_us=now;
  if(consumed_wheel && s.id-consumed_wheel>1U) stats[7]+=s.id-consumed_wheel-1U;
  consumed_wheel=s.id;
  uint8_t p[MAX_PAYLOAD]; uint16_t n=0;
  memcpy(p,epoch,16); n=16; put(p,&n,s.reference,4); put(p,&n,s.id,4);
  put(p,&n,s.t0,8); put(p,&n,s.t1,8);
  for(uint8_t i=0;i<2U;++i) put(p,&n,(uint32_t)s.velocity[i],4);
  for(uint8_t i=0;i<3U;++i) put(p,&n,s.raw[i],4);
  for(uint8_t i=0;i<3U;++i) put(p,&n,(uint32_t)s.delta[i],4);
  put(p,&n,s.flags,4);
  /* Rear EXTI edge latency and sensor acquisition delay have no proven bound. */
  put(p,&n,UINT32_MAX,4); put(p,&n,s.capture_us,4);
  maximum(16,DebugTelemetry_NowUs()-s.t1);
  (void)send_frame(WHEEL,packet_sequence++,p,n);
}

static void reply_request(const Request *r)
{
  const uint8_t *b=r->bytes;
  const uint16_t length=(uint16_t)read_be(b+4,2), seq=(uint16_t)read_be(b+6,2);
  if(b[r->size-1U]!=0xe4U || crc16(b+2,length+6U)!=(uint16_t)(b[length+8U]|b[length+9U]<<8)) {
    ++stats[1]; return;
  }
  ++stats[0];
  const uint8_t type=b[3];
  const uint64_t nonce=length>=8U ? read_be(b+8,8):0U;
  uint16_t status=b[2]!=1U ? BAD_VERSION :
      (type!=ID_REQ && type!=SYNC_REQ && type!=CONFIG_REQ && type!=STATS_REQ) ? BAD_TYPE :
      length!=(type==CONFIG_REQ ? 12U:8U) ? BAD_LENGTH:OK;
  uint8_t p[MAX_PAYLOAD]; uint16_t n=0;
  put(p,&n,nonce,8);
  if(status!=OK) {
    put(p,&n,status,2); put(p,&n,type,1); put(p,&n,1U,1);
    (void)send_frame(ERROR_REPLY,seq,p,n); return;
  }
  if(type==ID_REQ) {
    ++stats[22];
    put(p,&n,epoch_valid?0U:1U,2); memcpy(p+n,epoch,16); n+=16;
    put(p,&n,reference_epoch,4); put(p,&n,0x3ffU,8);
    put(p,&n,405U,2); put(p,&n,APP_ACTIVE_MODE,1); put(p,&n,1U,1);
    put(p,&n,1U,1); put(p,&n,0U,1); put(p,&n,1000000U,4);
    put(p,&n,huart3.Init.BaudRate,4); put(p,&n,epoch_valid?3U:1U,4);
    put(p,&n,UINT32_MAX,4);
    for(uint8_t i=0;i<3U;++i) put(p,&n,((const uint32_t *)UID_BASE)[i],4);
    put(p,&n,DBGMCU->IDCODE,4); put(p,&n,SCB->CPUID,4);
    put(p,&n,*(const uint16_t *)FLASHSIZE_BASE,2);
    memcpy(p+n,firmware_id,32); n+=32; memcpy(p+n,git_id,20); n+=20;
    put(p,&n,DEBUG_SOURCE_DIRTY,1); put(p,&n,MAX_PAYLOAD,2);
    (void)send_frame(ID_REPLY,seq,p,n);
  } else if(type==SYNC_REQ) {
    ++stats[21]; memcpy(p+n,epoch,16); n+=16;
    put(p,&n,r->received,8); put(p,&n,DebugTelemetry_NowUs(),8);
    put(p,&n,1U,4); put(p,&n,epoch_valid?1U:0U,2);
    (void)send_frame(SYNC_REPLY,seq,p,n);
  } else if(type==CONFIG_REQ) {
    const bool enabled=b[16]!=0U;
    const uint8_t profile=b[17];
    status=(b[16]>1U || profile!=1U || b[18] || b[19]) ? BAD_PROFILE :
        enabled && huart3.Init.BaudRate<REQUIRED_BAUD ? BANDWIDTH:OK;
    if(status==OK) {
      streaming=enabled;
      next_gyro_us=DebugTelemetry_NowUs();
      consumed_wheel=latest_wheel.id;
    }
    memcpy(p+n,epoch,16); n+=16; put(p,&n,reference_epoch,4);
    put(p,&n,status,2); put(p,&n,1U,1); put(p,&n,streaming,1);
    put(p,&n,huart3.Init.BaudRate,4); put(p,&n,WHEEL_PERIOD_US,4);
    put(p,&n,GYRO_PERIOD_US,4); put(p,&n,REQUIRED_BAUD,4);
    (void)send_frame(CONFIG_REPLY,seq,p,n);
  } else {
    memcpy(p+n,epoch,16); n+=16; put(p,&n,reference_epoch,4);
    put(p,&n,DebugTelemetry_NowUs(),8);
    uint32_t snapshot[24]; const uint32_t m=enter();
    for(uint8_t i=0;i<24U;++i) { snapshot[i]=stats[i]; }
    leave(m);
    for(uint8_t i=0;i<24U;++i) put(p,&n,snapshot[i],4);
    (void)send_frame(STATS_REPLY,seq,p,n);
  }
}

void DebugTelemetry_Process(void)
{
  if(!initialized) return;
  if(request_tail!=request_head) {
    Request r;
    const uint32_t m=enter(); r=requests[request_tail];
    request_tail=(uint8_t)((request_tail+1U)%REQUEST_SLOTS); leave(m);
    reply_request(&r);
  }
  if(!streaming) return;
  const uint64_t now=DebugTelemetry_NowUs();
  if(now>=next_gyro_us) {
    if(next_gyro_us && now-next_gyro_us>=GYRO_PERIOD_US) stats[10]+=bounded_u32((now-next_gyro_us)/GYRO_PERIOD_US);
    next_gyro_us=now+GYRO_PERIOD_US;
    publish_gyro(now);
  }
  publish_wheel();
}

void DebugTelemetry_TxStarted(const uint8_t *data, uint16_t size, uint64_t queued)
{
  if(size>=11U && data[0]==0xd4U && data[1]==0x4dU) {
    const uint64_t now=DebugTelemetry_NowUs();
    ++stats[12]; maximum(17,now-queued);
    if(data[3]==WHEEL && size==95U) { maximum(16,now-read_be(data+40,8)); }
    if(data[3]==GYRO && size==95U) {
      const uint64_t yaw=read_be(data+40,8),rate=read_be(data+64,8);
      if(yaw && rate) { maximum(23,now-(yaw<rate ? yaw:rate)); }
    }
  }
}
void DebugTelemetry_TxCompleted(const uint8_t *data, uint16_t size)
{
  if(size>=11U && data[0]==0xd4U && data[1]==0x4dU) ++stats[13];
}
void DebugTelemetry_TxDropped(void) { ++stats[14]; }
#endif
