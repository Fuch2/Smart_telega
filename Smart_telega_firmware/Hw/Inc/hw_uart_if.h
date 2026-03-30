#ifndef HW_UART_IF_H
#define HW_UART_IF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void     hw_uart_if_init(void);
void     hw_uart_if_step(void);          /* вызывать из mainloop */
bool     hw_uart_if_rx_byte_ready(void);
uint8_t  hw_uart_if_rx_byte_take(void);
void     hw_uart_if_tx(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* HW_UART_IF_H */
