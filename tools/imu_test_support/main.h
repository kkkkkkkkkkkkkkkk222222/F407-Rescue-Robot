#ifndef IMU_TEST_MAIN_H
#define IMU_TEST_MAIN_H
#include <stdint.h>
typedef struct { int unused; } SPI_HandleTypeDef;
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define IMU_CS_GPIO_Port 0
#define IMU_CS_Pin 0
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void HAL_GPIO_WritePin(int p, int n, int v) { (void)p; (void)n; (void)v; }
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
#endif
