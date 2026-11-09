#include "servo.h"
#include "main.h"
#include "TaskDiag.h"

/* Lift/right claw/camera/left claw -> JP304/JP307/JP308/JP309. */
static const uint16_t pins[4] = {GPIO_PIN_13, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2};
static volatile uint8_t angles[4];
static int16_t offsets_deg[4] = {
    APP_SERVO_S1_OFFSET_DEG, APP_SERVO_S2_OFFSET_DEG,
    APP_SERVO_S3_OFFSET_DEG, APP_SERVO_S4_OFFSET_DEG
};
static bool started;
static volatile uint16_t pending_pins, armed_pins;

enum { SERVO_SLOT_US = 5000U, SERVO_FIRST_EDGE_US = 100U, SERVO_EDGES = 8U };
static volatile uint32_t edge_ticks[SERVO_EDGES];
static volatile uint32_t edge_bsrr[SERVO_EDGES];

static uint32_t servo_slot_start(uint8_t index)
{
  return SERVO_FIRST_EDGE_US + (uint32_t)index * SERVO_SLOT_US;
}

uint8_t Servo_OutputAngle(uint8_t id, uint8_t angle)
{
  if (id < 1U || id > 4U) { return 0U; }
  if (angle > 180U) { angle = 180U; }
  int32_t output_deg = (int32_t)angle + offsets_deg[id - 1U];
  if (output_deg < 0) { output_deg = 0; }
  if (output_deg > 180) { output_deg = 180; }
  return (uint8_t)output_deg;
}

static uint32_t servo_pulse_for_angle(uint8_t index, uint8_t angle)
{
  return 500U + (uint32_t)Servo_OutputAngle(index + 1U, angle) * 2000U / 180U;
}

void BoardF405_ServoIRQ(void)
{
  if ((TIM4->SR & TIM_SR_UIF) == 0U) { return; }
  TIM4->SR = ~TIM_SR_UIF;
  /* Stage a newly enabled test channel across a frame so its first pulse
   * uses the new width even if the timing DMA prefetched an old word. */
  for (uint8_t i = 0; i < 4U; ++i) {
    if (armed_pins & pins[i]) { edge_bsrr[2U * i] = pins[i]; }
  }
  armed_pins = pending_pins;
  pending_pins = 0U;
  if (armed_pins == 0U) { TIM4->DIER &= ~TIM_DIER_UIE; }
}

static void servo_start(uint16_t outputs)
{
  if (started) { BoardF405_ServoEmergencyOff(); }
  HAL_NVIC_DisableIRQ(TIM4_IRQn);
  pending_pins = 0U;
  armed_pins = 0U;
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  TIM4->CR1 = 0U;
  TIM4->DIER = 0U;
  for (uint8_t i = 0; i < 4U; ++i) {
    angles[i] = 90U;
    edge_ticks[2U * i] = servo_slot_start(i) + servo_pulse_for_angle(i, 90U);
    edge_ticks[2U * i + 1U] = servo_slot_start((i + 1U) % 4U);
    edge_bsrr[2U * i] = outputs & pins[i];
    edge_bsrr[2U * i + 1U] = (uint32_t)pins[i] << 16U;
  }

  /* Each compare advances CCR1 to the next edge. TRGO sends the same event
   * through TIM1 ITR3/TRC to spare capture channel 3. Encoder CH1/2 stay in
   * quadrature mode. Only DMA writes the signal edges to GPIOC. */
  TIM4->CR2 = TIM_TRGO_OC1;
  TIM4->PSC = 83U; /* APB1 timer 84 MHz -> 1 us per tick. */
  TIM4->ARR = 19999U;
  TIM4->CNT = 0U;
  TIM4->CCMR1 = 0U;
  TIM4->CCMR2 = 0U;
  TIM4->CCER = TIM_CCER_CC1E;
  TIM4->CCR1 = servo_slot_start(0U);
  TIM4->EGR = TIM_EGR_UG;
  TIM4->SR = 0U;

  TIM1->CCER &= ~TIM_CCER_CC3E;
  TIM1->SMCR = (TIM1->SMCR & ~TIM_SMCR_TS) | TIM_TS_ITR3;
  TIM1->CCMR2 = (TIM1->CCMR2 & ~(TIM_CCMR2_CC3S | TIM_CCMR2_IC3PSC |
                                TIM_CCMR2_IC3F)) | TIM_CCMR2_CC3S;
  TIM1->CCER = (TIM1->CCER & ~(TIM_CCER_CC3P | TIM_CCER_CC3NP)) | TIM_CCER_CC3E;
  TIM1->SR = ~(TIM_SR_CC3IF | TIM_SR_CC3OF);

  /* RM0090: TIM4_CC1 on DMA1 stream 0/channel 2. */
  DMA1->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 |
                DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
  DMA1_Stream0->PAR = (uint32_t)(uintptr_t)&TIM4->CCR1;
  DMA1_Stream0->M0AR = (uint32_t)(uintptr_t)edge_ticks;
  DMA1_Stream0->NDTR = SERVO_EDGES;
  DMA1_Stream0->FCR = 0U;
  DMA1_Stream0->CR = DMA_CHANNEL_2 | DMA_MEMORY_TO_PERIPH | DMA_MINC_ENABLE |
                     DMA_PDATAALIGN_WORD | DMA_MDATAALIGN_WORD | DMA_CIRCULAR |
                     DMA_PRIORITY_HIGH;

  /* TIM1_CC3 on DMA2 stream 6/channel 0. ES0182 calls for the memory port
   * on GPIO writes while an APB2 peripheral (USART1 RX) also uses DMA2. */
  DMA2->HIFCR = DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6 |
                DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6;
  DMA2_Stream6->PAR = (uint32_t)(uintptr_t)edge_bsrr;
  DMA2_Stream6->M0AR = (uint32_t)(uintptr_t)&GPIOC->BSRR;
  DMA2_Stream6->NDTR = SERVO_EDGES;
  DMA2_Stream6->FCR = 0U;
  DMA2_Stream6->CR = DMA_CHANNEL_0 | DMA_PERIPH_TO_MEMORY | DMA_PINC_ENABLE |
                     DMA_PDATAALIGN_WORD | DMA_MDATAALIGN_WORD | DMA_CIRCULAR |
                     DMA_PRIORITY_HIGH;
  GPIOC->BSRR = (uint32_t)F405_SERVO_PINS << 16U;
  __DMB();
  DMA1_Stream0->CR |= DMA_SxCR_EN;
  DMA2_Stream6->CR |= DMA_SxCR_EN;
  TIM1->DIER |= TIM_DIER_CC3DE;
  started = true;
  HAL_NVIC_SetPriority(TIM4_IRQn, 1U, 0U);
  HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);
  TIM4->DIER = TIM_DIER_CC1DE;
  TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void Servo_Init(void) { servo_start(F405_SERVO_PINS); }

void BoardF405_ServoTestSet(uint8_t id, uint8_t angle)
{
  if (id < 1U || id > 4U || angle > 180U) { return; }
  if (!started) { servo_start(0U); }
  Servo_SetAngle(id, angle);
  const uint8_t index = id - 1U;
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (edge_bsrr[2U * index] == 0U) {
    armed_pins &= (uint16_t)~pins[index];
    pending_pins |= pins[index];
    if ((TIM4->DIER & TIM_DIER_UIE) == 0U) {
      TIM4->SR = ~TIM_SR_UIF;
      TIM4->DIER |= TIM_DIER_UIE;
    }
  }
  __set_PRIMASK(mask);
}

void BoardF405_ServoTestOff(uint8_t id)
{
  if (id > 4U) { return; }
  const uint16_t off = id == 0U ? F405_SERVO_PINS : pins[id - 1U];
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  pending_pins &= (uint16_t)~off;
  armed_pins &= (uint16_t)~off;
  /* Leave an in-flight pulse to its scheduled falling edge. */
  for (uint8_t i = 0; i < 4U; ++i) {
    if (off & pins[i]) { edge_bsrr[2U * i] = 0U; }
  }
  __set_PRIMASK(mask);
}

void Servo_SetAngle(uint8_t id, uint8_t angle)
{
  if (id < 1U || id > 4U) { return; }
  if (angle > 180U) { angle = 180U; }
  TaskDiag_Event("SERVO_TARGET", id, angle, 0, 0, 0, 0);
  const uint8_t index = id - 1U;
  angles[index] = angle;
  if (started) {
    /* An aligned word sets the next pulse. A loaded CCR1 stays fixed. */
    edge_ticks[2U * index] = servo_slot_start(index) + servo_pulse_for_angle(index, angle);
  }
}

uint8_t Servo_GetAngle(uint8_t id)
{
  return id >= 1U && id <= 4U ? angles[id - 1U] : 0U;
}

void Servo_SetOffset(uint8_t id, int16_t offset_deg)
{
  if (id < 1U || id > 4U) { return; }
  offsets_deg[id - 1U] = offset_deg;
  Servo_SetAngle(id, Servo_GetAngle(id));
}

void BoardF405_ServoEmergencyOff(void)
{
  if (!started) { return; }
  TIM4->CR1 = 0U;
  TIM4->DIER = 0U;
  TIM1->DIER &= ~TIM_DIER_CC3DE;
  DMA1_Stream0->CR &= ~DMA_SxCR_EN;
  DMA2_Stream6->CR &= ~DMA_SxCR_EN;
  while ((DMA1_Stream0->CR | DMA2_Stream6->CR) & DMA_SxCR_EN) {}
  GPIOC->BSRR = (uint32_t)F405_SERVO_PINS << 16U;
  started = false;
}
