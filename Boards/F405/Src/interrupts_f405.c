#include "main.h"
#include "Robot.h"
#include "imu.h"

void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) { Robot_RunDeferredTask(); }
void SysTick_Handler(void) { HAL_IncTick(); }
void TIM6_DAC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim6); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void UART4_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }
void UART5_IRQHandler(void) { IMU_UART_IRQHandler(); }
void DMA1_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart3_rx); }
void EXTI9_5_IRQHandler(void) { BoardF405_EncoderIRQ(); }
void TIM4_IRQHandler(void) { BoardF405_ServoIRQ(); }
