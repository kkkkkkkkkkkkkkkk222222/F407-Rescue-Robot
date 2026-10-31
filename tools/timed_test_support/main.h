#ifndef TIMED_TEST_MAIN_H
#define TIMED_TEST_MAIN_H
#include <stdint.h>
typedef struct { struct { uint32_t BaudRate; } Init; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart3;
extern uint32_t fake_uid[3];
#define UID_BASE ((uintptr_t)fake_uid)
static const struct { uint32_t IDCODE; } fake_debug={0x100f0413};
static const struct { uint32_t CPUID; } fake_cpu={0x410fc241};
static const uint16_t fake_flash=1024;
#define DBGMCU (&fake_debug)
#define SCB (&fake_cpu)
#define FLASHSIZE_BASE ((uintptr_t)&fake_flash)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
static inline void __DMB(void) {}
#endif
