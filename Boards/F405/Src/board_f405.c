#include "main.h"
#include "Robot.h"
#include "MotionDiag.h"

#if !APP_IMU_USE_UART
#error "NormalDebug uses the external UART gyro, not the old SPI IMU660RC"
#endif

UART_HandleTypeDef huart1, huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_console_rx;
DMA_HandleTypeDef hdma_debug_upper_tx;
TIM_HandleTypeDef htim6, htim8, htim1;
TIM_HandleTypeDef board_motor_timers[3];
const uint32_t board_motor_channels[3] = {TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_1};

static void check(HAL_StatusTypeDef result)
{
  if (result != HAL_OK) { Error_Handler(); }
}

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(PendSV_IRQn, 15U, 0U);
}

static void system_clock(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 8U;
  osc.PLL.PLLN = 336U;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 7U;
  check(HAL_RCC_OscConfig(&osc));
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4;
  clk.APB2CLKDivider = RCC_HCLK_DIV2;
  check(HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5));
}

static void gpio(GPIO_TypeDef *port, uint16_t pins, uint32_t mode,
                 uint32_t pull, uint32_t alternate)
{
  GPIO_InitTypeDef init = {0};
  init.Pin = pins;
  init.Mode = mode;
  init.Pull = pull;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  init.Alternate = alternate;
  HAL_GPIO_Init(port, &init);
}

static void board_gpio(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                           GPIO_PIN_4 | GPIO_PIN_7 | GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | F405_SERVO_PINS, GPIO_PIN_RESET);
  gpio(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
              GPIO_PIN_7 | GPIO_PIN_10, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0U);
  gpio(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0U);
  gpio(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | F405_SERVO_PINS,
       GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0U);
  gpio(GPIOB, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13,
       GPIO_MODE_INPUT, GPIO_PULLUP, 0U);
  /* Console uses this whole connector. Do NOT bit-bang I2C here: the attached
   * programmer TX actively drives PB7. The onboard IMU remains physically shared. */
  gpio(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_INPUT, GPIO_PULLUP, 0U);
}

static void motor_timer(TIM_HandleTypeDef *timer, TIM_TypeDef *instance, uint32_t channel)
{
  TIM_OC_InitTypeDef pwm = {0};
  timer->Instance = instance;
  timer->Init.Prescaler = 0U;
  timer->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer->Init.Period = 4199U; /* 84 MHz / 4200 = 20 kHz. */
  timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  check(HAL_TIM_PWM_Init(timer));
  pwm.OCMode = TIM_OCMODE_PWM1;
  pwm.Pulse = 0U;
  pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm.OCFastMode = TIM_OCFAST_DISABLE;
  check(HAL_TIM_PWM_ConfigChannel(timer, &pwm, channel));
}

static void encoder_timer(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
{
  TIM_Encoder_InitTypeDef enc = {0};
  timer->Instance = instance;
  timer->Init.Prescaler = 0U;
  timer->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer->Init.Period = 65535U;
  timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  enc.EncoderMode = TIM_ENCODERMODE_TI12;
  enc.IC1Polarity = enc.IC2Polarity = TIM_ICPOLARITY_RISING;
  enc.IC1Selection = enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC1Prescaler = enc.IC2Prescaler = TIM_ICPSC_DIV1;
  enc.IC1Filter = enc.IC2Filter = 4U;
  check(HAL_TIM_Encoder_Init(timer, &enc));
}

static void board_timers(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM6_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM13_CLK_ENABLE();
  /* Logical wheels use physical sockets 4, 2, 3. Separate PWM timers avoid
   * resetting another motor's PWM phase when committing a direction change. */
  motor_timer(&board_motor_timers[0], TIM3, board_motor_channels[0]);
  motor_timer(&board_motor_timers[1], TIM2, board_motor_channels[1]);
  motor_timer(&board_motor_timers[2], TIM13, board_motor_channels[2]);
  gpio(GPIOB, GPIO_PIN_1, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_AF2_TIM3);
  gpio(GPIOA, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_AF1_TIM2);
  gpio(GPIOA, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_AF9_TIM13);
  gpio(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1);
  gpio(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_MODE_INPUT, GPIO_PULLUP, 0U);
  gpio(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8);
  encoder_timer(&htim1, TIM1);
  encoder_timer(&htim8, TIM8);
  /* TIM8 CH3/CH4 cannot be another independent quadrature encoder. */
  gpio(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLUP, 0U);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2U, 0U);
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83U;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999U;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  check(HAL_TIM_Base_Init(&htim6));
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
  if (uart->Instance == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    gpio(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART1);
#if APP_ENABLE_MOTION_DEBUG_TASK
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_console_rx.Instance = DMA2_Stream2;
    hdma_console_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_console_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_console_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_console_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_console_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_console_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_console_rx.Init.Mode = DMA_CIRCULAR;
    hdma_console_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_console_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    check(HAL_DMA_Init(&hdma_console_rx));
    __HAL_LINKDMA(uart, hdmarx, hdma_console_rx);
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
#endif
  } else if (uart->Instance == UART4) {
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    gpio(GPIOC, GPIO_PIN_10 | GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART4);
    hdma_usart3_rx.Instance = DMA1_Stream2;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    check(HAL_DMA_Init(&hdma_usart3_rx));
    __HAL_LINKDMA(uart, hdmarx, hdma_usart3_rx);
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
#if APP_ENABLE_MOTION_DEBUG_TASK
    hdma_debug_upper_tx.Instance = DMA1_Stream4;
    hdma_debug_upper_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_debug_upper_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_debug_upper_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_debug_upper_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_debug_upper_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_debug_upper_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_debug_upper_tx.Init.Mode = DMA_NORMAL;
    hdma_debug_upper_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_debug_upper_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    check(HAL_DMA_Init(&hdma_debug_upper_tx));
    __HAL_LINKDMA(uart, hdmatx, hdma_debug_upper_tx);
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
#endif
  } else if (uart->Instance == UART5) {
    __HAL_RCC_UART5_CLK_ENABLE();
    gpio(GPIOC, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART5);
    gpio(GPIOD, GPIO_PIN_2, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_AF8_UART5);
  }
}

static void uart_init(UART_HandleTypeDef *uart, USART_TypeDef *instance)
{
  uart->Instance = instance;
  uart->Init.BaudRate = 115200U;
#if APP_ENABLE_MOTION_DEBUG_TASK
  if (instance == UART4) { uart->Init.BaudRate = APP_NORMALDEBUG_UPPER_BAUD; }
#endif
  uart->Init.WordLength = UART_WORDLENGTH_8B;
  uart->Init.StopBits = UART_STOPBITS_1;
  uart->Init.Parity = UART_PARITY_NONE;
  uart->Init.Mode = UART_MODE_TX_RX;
  uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart->Init.OverSampling = UART_OVERSAMPLING_16;
  check(HAL_UART_Init(uart));
}

int main(void)
{
  HAL_Init();
  system_clock();
  board_gpio();
  board_timers();
  uart_init(&huart1, USART1);
  uart_init(&huart3, UART4);
  Robot_Init();
  BoardF405_ReportVersion();
  while (1) { Robot_Process(); }
}

void Error_Handler(void)
{
#if APP_ENABLE_MOTION_DEBUG_TASK
  MotionDiag_Error((uint32_t)__builtin_return_address(0));
#else
  /* Initialization failure, same fatal path as the original board. */
  __disable_irq();
  GPIOA->BSRR = (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_7) << 16U;
  GPIOB->BSRR = GPIO_PIN_0 << 16U;
  GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5 | F405_SERVO_PINS) << 16U;
  TIM2->CCER = TIM3->CCER = TIM13->CCER = 0U;
  while (1) {}
#endif
}
