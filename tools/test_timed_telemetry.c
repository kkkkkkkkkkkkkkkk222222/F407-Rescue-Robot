#include "main.h"
#include "DebugTelemetry.h"
#include "imu.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
UART_HandleTypeDef huart3={.Init.BaudRate=230400};
uint32_t fake_uid[3]={1,2,3};
static uint64_t now;
static unsigned boots, count;
static uint8_t frames[128][192];
static uint16_t sizes[128];
static IMUData imu={.ready=true};
uint64_t DebugTelemetry_NowUs(void) { return now; }
void DebugClock_Init(void) { now=100; }
bool DebugClock_NewEpoch(uint8_t epoch[16]) { memset(epoch,++boots,16); return true; }
IMUData IMU_GetData(void) { return imu; }
bool Uart_Send(const uint8_t *data,uint16_t size)
{ assert(count<128 && size<=192); memcpy(frames[count],data,size); sizes[count++]=size; return true; }
static uint16_t crc(const uint8_t *b,unsigned n)
{ uint16_t c=0xffff; for(unsigned i=0;i<n;i++){c^=b[i];for(unsigned j=0;j<8;j++)c=(c>>1)^((c&1)?0xa001:0);}return c; }
static void request(unsigned kind,unsigned nonce,unsigned version,bool enable,bool corrupt)
{
  unsigned n=kind==5?12:8;
  uint8_t frame[32]={0xd4,0x4d,version,kind,0,n,0,nonce};
  frame[15]=nonce;
  if(kind==5) { frame[16]=enable;frame[17]=1; }
  unsigned c=crc(frame+2,n+6);frame[n+8]=c;frame[n+9]=c>>8;frame[n+10]=0xe4;
  if(corrupt) frame[n+8]^=1;
  for(unsigned i=0;i<n+11;i++) { now+=44;assert(DebugTelemetry_RxByte(frame[i],true)); }
  now+=100;DebugTelemetry_Process();
}
static uint64_t get(const uint8_t *p,unsigned n)
{ uint64_t v=0;while(n--)v=(v<<8)|*p++;return v; }
static int latest(unsigned type)
{ for(int i=(int)count-1;i>=0;i--)if(frames[i][3]==type)return i;return -1; }
static void wheel(uint64_t t,const int32_t d[3],uint32_t bad,bool pending)
{
  static int64_t raw[3];for(unsigned i=0;i<3;i++)raw[i]+=d[i];
  now=t+2;DebugTelemetry_WheelSample(t,t+2,raw,d,bad,pending);DebugTelemetry_Process();
}
int main(void)
{
  DebugTelemetry_Init();
  assert(!DebugTelemetry_RxByte(0xd4,false));
  assert(!DebugTelemetry_RxByte(0xa3,true));
  assert(DebugTelemetry_RxByte(0xd4,true));now+=21000;
  assert(!DebugTelemetry_RxByte(0xa3,true));
  request(1,1,1,false,false);assert(sizes[latest(2)]==148);
  const uint8_t original_epoch=frames[latest(2)][18];
  request(3,2,1,false,false);assert(sizes[latest(4)]==57);
  assert(get(frames[latest(4)]+32,8)<get(frames[latest(4)]+40,8));
  request(1,3,9,false,false);assert(get(frames[latest(127)]+16,2)==1);
  unsigned old=count;request(1,4,1,false,true);assert(count==old);
  huart3.Init.BaudRate=115200;request(5,5,1,true,false);
  assert(get(frames[latest(6)]+36,2)==4);
  huart3.Init.BaudRate=230400;request(5,6,1,true,false);
  assert(get(frames[latest(6)]+36,2)==0);
  const int32_t z[3]={0,0,0},forward[3]={10,-10,0},left[3]={10,10,-20};
  wheel(10000,z,0,false);wheel(20000,forward,0,false);wheel(30000,forward,0,false);
  int w=latest(7);assert(w>=0 && sizes[w]==95);
  assert((int32_t)get(frames[w]+48,4)>0 && (int32_t)get(frames[w]+52,4)==0);
  assert(get(frames[w]+84,4)==UINT32_MAX);
  assert((get(frames[w]+80,4)&3)==1);
  wheel(40000,left,0,false);wheel(50000,left,0,false);
  w=latest(7);assert((int32_t)get(frames[w]+48,4)==0 && (int32_t)get(frames[w]+52,4)>0);
  now=61000;DebugTelemetry_GyroSample(0xbb,90000,30000,now-100);
  DebugTelemetry_GyroSample(0xaa,90000,30000,now-50);DebugTelemetry_Process();
  int g=latest(8);assert(g>=0 && sizes[g]==95);
  assert(get(frames[g]+52,4)&64);assert(get(frames[g]+76,4)&64);
  assert(get(frames[g]+40,8)!=get(frames[g]+64,8));
  now=71000;DebugTelemetry_Process();g=latest(8);assert(get(frames[g]+52,4)&128);
  wheel(80000,z,1,true);wheel(90000,z,1,false);
  w=latest(7);assert(!(get(frames[w]+80,4)&1));assert((int32_t)get(frames[w]+48,4)==INT32_MIN);
  now=100000;DebugTelemetry_ReferenceReset();
  wheel(100000,z,1,false);wheel(110000,z,1,false);
  request(1,7,1,false,false);assert(get(frames[latest(2)]+34,4)==1);
  request(9,8,1,false,false);assert(sizes[latest(10)]==143);
  wheel(120000,z,1,false);wheel(130000,z,1,false);
  w=latest(7);assert(get(frames[w]+80,4)&1);
  assert(get(frames[w]+48,4)==0 && get(frames[w]+52,4)==0);
  const int32_t reverse[3]={-10,10,0},right[3]={-10,-10,20};
  wheel(140000,reverse,1,false);wheel(150000,reverse,1,false);
  w=latest(7);assert((int32_t)get(frames[w]+48,4)<0);
  wheel(160000,right,1,false);wheel(170000,right,1,false);
  w=latest(7);assert((int32_t)get(frames[w]+52,4)<0);
  now=181000;DebugTelemetry_GyroSample(0xbb,-90000,-30000,now-100);
  DebugTelemetry_GyroSample(0xaa,-90000,-30000,now-50);DebugTelemetry_Process();
  g=latest(8);assert((int64_t)get(frames[g]+32,8)==-90000);
  assert((int32_t)get(frames[g]+60,4)==-30000);
  now=192000;DebugTelemetry_GyroSample(0xaa,-90000,2000000,now-50);DebugTelemetry_Process();
  g=latest(8);assert((get(frames[g]+76,4)&33)==32);
  request(5,9,1,false,false);old=count;now+=100000;DebugTelemetry_Process();assert(count==old);
  DebugTelemetry_Init();request(1,10,1,false,false);
  assert(frames[latest(2)][18]!=original_epoch);
  for(unsigned f=0;f<count;f++){
    assert(crc(frames[f]+2,sizes[f]-5)==(frames[f][sizes[f]-3]|frames[f][sizes[f]-2]<<8));
    printf("FRAME ");for(unsigned j=0;j<sizes[f];j++)printf("%02X",frames[f][j]);puts("");
  }
  puts("C firmware module: queries, baud gate, encoder feedback, invalid samples, gyro source identity, repeat, reference and reboot checks passed. HOST SIMULATION ONLY.");
}
