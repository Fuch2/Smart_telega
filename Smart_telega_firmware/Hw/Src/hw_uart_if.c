#include "hw_uart_if.h"
#include "main.h"

#include <string.h>

#define RX_RING_SIZE  256u

static uint8_t  g_rx_ring[RX_RING_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;

static uint8_t g_rx_it_byte;

extern UART_HandleTypeDef huart1;

/* ── init ────────────────────────────────────────────────────── */
void hw_uart_if_init(void)
{
    g_rx_head = 0;
    g_rx_tail = 0;
    HAL_UART_Receive_IT(&huart1, &g_rx_it_byte, 1);
}

/* ── HAL callback ────────────────────────────────────────────── */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint16_t next = (g_rx_head + 1u) % RX_RING_SIZE;
        if (next != g_rx_tail) {
            g_rx_ring[g_rx_head] = g_rx_it_byte;
            g_rx_head = next;
        }
        HAL_UART_Receive_IT(&huart1, &g_rx_it_byte, 1);
    }
}

/* ── API ─────────────────────────────────────────────────────── */

/* ПРАВКА 1: hw_uart_if_step вынесен в отдельную секцию —
   тело остаётся пустым, но готово к DMA/timeout расширению */
void hw_uart_if_step(void)
{

}


bool hw_uart_if_rx_byte_ready(void)
{
    return (g_rx_head != g_rx_tail);
}

/* ПРАВКА 2: defensive guard против чтения из пустого буфера */
uint8_t hw_uart_if_rx_byte_take(void)
{
    uint8_t b = 0u;
    if (g_rx_head != g_rx_tail) {
        b = g_rx_ring[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1u) % RX_RING_SIZE;
    }
    return b;
}

void hw_uart_if_tx(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0u)) { return; }
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
}
