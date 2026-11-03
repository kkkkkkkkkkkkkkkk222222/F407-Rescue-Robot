#ifndef UART_LINK_H
#define UART_LINK_H

#include <stdbool.h>
#include <stdint.h>

bool Uart_Init(void);
bool Uart_Send(const uint8_t *data, uint16_t size);
bool Uart_Receive(uint16_t size);

typedef struct {
  uint32_t rx_errors, last_rx_error, rx_restarts, tx_dropped;
  uint32_t odom_tx_frames, imu_tx_frames;
  uint16_t tx_remaining, tx_size;
  bool rx_active, rx_pending;
} UartLinkStatus;
UartLinkStatus Uart_GetLinkStatus(void);

#endif
