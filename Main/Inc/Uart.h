#ifndef UART_LINK_H
#define UART_LINK_H

#include <stdbool.h>
#include <stdint.h>

bool Uart_Init(void);
bool Uart_Send(const uint8_t *data, uint16_t size);
bool Uart_Receive(uint16_t size);

#endif
