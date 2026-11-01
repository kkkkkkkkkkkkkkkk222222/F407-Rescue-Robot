/* Actual Uart.c with a deterministic peripheral model; no serial/hardware I/O. */
#include "main.h"
#include "Uart.h"
#include "DebugTelemetry.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
USART_TypeDef console_regs, upper_regs={.SR=USART_SR_TC};
static DMA_Stream_TypeDef tx_regs,rx_regs;
static DMA_HandleTypeDef tx={.Instance=&tx_regs,.State=HAL_DMA_STATE_READY};
static DMA_HandleTypeDef rx={.Instance=&rx_regs,.State=HAL_DMA_STATE_READY};
UART_HandleTypeDef huart1={.Instance=&console_regs};
UART_HandleTypeDef huart3={.Instance=&upper_regs,.hdmatx=&tx,.hdmarx=&rx,.gState=HAL_UART_STATE_READY};
static uint64_t now=1000;
static const uint8_t *dma_data;
static uint16_t dma_size;
static unsigned starts,completed,aborts;
static HAL_StatusTypeDef next_start=HAL_OK;
static uint8_t wire[500000];
static size_t wire_size;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
uint64_t DebugTelemetry_NowUs(void) { return now; }
uint32_t Robot_GetMilliseconds(void) { return (uint32_t)(now/1000); }
void DebugTelemetry_TxDropped(void) {}
void DebugTelemetry_TxStarted(const uint8_t *d,uint16_t n,uint64_t t) { (void)d;(void)n;(void)t; }
void DebugTelemetry_TxCompleted(const uint8_t *d,uint16_t n) { (void)d;(void)n; }
void Vision_ResetParser(void) {}
void Vision_ParseBytes(const uint8_t *d,size_t n,uint32_t t) { (void)d;(void)n;(void)t; }
void WheelCheck_OnRxEvent(void) {}
void WheelCheck_OnTxComplete(void) {}
void WheelCheck_OnUartError(uint32_t e) { (void)e; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ (void)h;(void)p;(void)n;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h;return HAL_OK; }
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *h) { h->State=HAL_DMA_STATE_READY;h->Instance->CR=0;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *h)
{
  ++aborts; h->Instance->CR3&=~USART_CR3_DMAT;
  h->Instance->CR1&=~(USART_CR1_TCIE|USART_CR1_TXEIE);
  h->gState=HAL_UART_STATE_READY; tx.State=HAL_DMA_STATE_READY;tx_regs.CR=0;
  return HAL_OK; /* Deliberately does not drain the UART shifter. */
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,const uint8_t *p,uint16_t n)
{
  HAL_StatusTypeDef result=next_start;next_start=HAL_OK;
  if(result!=HAL_OK) return result;
  assert(!dma_data && !(tx_regs.CR&DMA_SxCR_EN));
  assert(!(h->Instance->CR3&USART_CR3_DMAT));
  dma_data=p;dma_size=n;++starts;
  h->gState=HAL_UART_STATE_BUSY_TX;h->Instance->SR&=~USART_SR_TC;
  h->Instance->CR3|=USART_CR3_DMAT;tx_regs.CR=DMA_SxCR_EN;
  tx.State=HAL_DMA_STATE_BUSY;tx_regs.NDTR=n;
  return HAL_OK;
}
static void finish(void)
{
  assert(dma_data && wire_size+dma_size<sizeof(wire));
  memcpy(wire+wire_size,dma_data,dma_size);wire_size+=dma_size;
  now+=(uint64_t)dma_size*44U;dma_data=NULL;
  tx_regs.CR=0;tx_regs.NDTR=0;tx.State=HAL_DMA_STATE_READY;
  upper_regs.CR3&=~USART_CR3_DMAT;upper_regs.SR|=USART_SR_TC;
  huart3.gState=HAL_UART_STATE_READY;
  HAL_UART_TxCpltCallback(&huart3);++completed;
}
static uint16_t crc(const uint8_t *p,unsigned n)
{ uint16_t c=0xffff;while(n--){c^=*p++;for(unsigned b=0;b<8;b++)c=(c>>1)^((c&1)?0xa001:0);}return c; }
static unsigned frame(uint8_t *p,unsigned type,unsigned seq)
{
  unsigned n=type>=7U?95U:15U;
  memset(p,0,n);
  if(n==15) {
    p[0]=0xa3;p[1]=0xb3;p[2]=type==1?0x15:0x1f;p[3]=seq;
    uint16_t c=crc(p+2,10);p[12]=c;p[13]=c>>8;p[14]=0xc3;
  } else {
    p[0]=0xd4;p[1]=0x4d;p[2]=1;p[3]=type;p[5]=84;p[6]=seq>>8;p[7]=seq;
    uint16_t c=crc(p+2,90);p[92]=c;p[93]=c>>8;p[94]=0xe4;
  }
  return n;
}
int main(void)
{
  assert(Uart_Init());uint8_t p[192],expected[500000];size_t expected_size=0;
  /* Mixed framing, caller-buffer reuse, and completion callbacks cannot start DMA. */
  for(unsigned i=0;i<2000;i++) {
    unsigned n=frame(p,(i%4U)+5U>=7U?(i%2U)+7U:(i%2U)+1U,i);
    memcpy(expected+expected_size,p,n);expected_size+=n;
    unsigned before=starts;assert(Uart_Send(p,n));memset(p,0xee,n);assert(starts==before);
    assert(Uart_Receive(0));assert(starts==before+1U);finish();assert(starts==before+1U);
  }
  assert(wire_size==expected_size && !memcmp(wire,expected,wire_size));
  uint32_t q[30];Uart_GetTimedQuality(q);assert(q[0]==2000 && q[1]==2000 && !q[7]);
  /* Queued same-type replacement must not touch the active dedicated buffer. */
  unsigned n=frame(p,7,3000);assert(Uart_Send(p,n));Uart_Receive(0);
  uint8_t active[95];memcpy(active,dma_data,n);
  frame(p,7,3001);assert(Uart_Send(p,n));frame(p,7,3002);assert(Uart_Send(p,n));
  assert(!memcmp(active,dma_data,n));finish();Uart_Receive(0);finish();
  Uart_GetTimedQuality(q);assert(q[11]>=1);
  /* Expiration is a visible drop, not old feedback retransmitted as new. */
  frame(p,8,3003);assert(Uart_Send(p,n));now+=9000;unsigned before=starts;
  Uart_Receive(0);assert(starts==before);Uart_GetTimedQuality(q);assert(q[10]>=1);
  /* Even HAL_BUSY must release ownership only through explicit recovery. */
  frame(p,1,3);assert(Uart_Send(p,15));next_start=HAL_BUSY;
  Uart_Receive(0);Uart_Receive(0);assert(aborts>=1);
  frame(p,2,4);assert(Uart_Send(p,15));Uart_Receive(0);finish();
  /* Abort leaves up to two USART bytes in flight: block restart until TC. */
  frame(p,1,5);assert(Uart_Send(p,15));Uart_Receive(0);
  now+=101000;Uart_Receive(0);assert(aborts>=2);
  dma_data=NULL;before=starts;
  frame(p,2,6);assert(Uart_Send(p,15));Uart_Receive(0);assert(starts==before);
  upper_regs.SR|=USART_SR_TC;Uart_Receive(0);assert(starts==before+1);finish();
  HAL_UART_TxCpltCallback(&huart3);Uart_GetTimedQuality(q);assert(q[6]>=1);
  /* A stale request gate must not clock even the first two bytes of a frame. */
  upper_regs.CR3|=USART_CR3_DMAT;frame(p,1,7);assert(Uart_Send(p,15));
  before=starts;Uart_Receive(0);assert(starts==before);Uart_Receive(0);
  Uart_Receive(0);finish();Uart_GetTimedQuality(q);assert(q[8]>=1);
  /* A corrupted active buffer is reported, never counted as successful TC. */
  frame(p,1,8);assert(Uart_Send(p,15));Uart_Receive(0);
  ((uint8_t *)dma_data)[8]^=1U;finish();Uart_Receive(0);
  Uart_GetTimedQuality(q);assert(q[7]==1);
  printf("HOST MODEL ONLY: %u completed transfers; 2000 mixed CRC frames byte-exact, caller reuse, coalescing, expiry, HAL_BUSY, abort-drain, unexpected TC passed.\n",completed);
  printf("TX started=%u complete=%u recoveries=%u seal_errors=%u precondition=%u expired=%u replaced=%u bytes_started=%u bytes_completed=%u\n",q[0],q[1],q[4],q[7],q[8],q[10],q[11],q[13],q[14]);
}
