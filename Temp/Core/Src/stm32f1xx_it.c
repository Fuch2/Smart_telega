/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
#define UART_RX_BUF_SIZE 64

uint8_t  rx_byte;                       // один байт для HAL_UART_Receive_IT
char     rx_buf[UART_RX_BUF_SIZE];      // буфер для одного кадра
uint8_t  rx_pos   = 0;                  // текущая позиция записи
uint8_t  in_frame = 0;                  // 0 - ждём STX, 1 - внутри кадра

extern volatile uint8_t  measuring_enabled;
extern volatile uint32_t T_ms;

// Эти переменные объявлены в main.c (убрать у них static!)
extern uint8_t smm_N;
extern uint8_t smm_count;
extern uint8_t smm_index;

static float alpha = 0.5f;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

// Структура "Команда + данные"
typedef struct {
    char *cmd;
    char *data;
} CommandFrame;

// Разделение строки на команду и данные
static uint8_t split_frame(char *buf, CommandFrame *out)
{
    char *p = buf;

    // пропускаем начальные пробелы
    while (*p == ' ') p++;
    if (*p == '\0') return 0;

    out->cmd = p;

    // до первого пробела - команда
    while (*p && *p != ' ') p++;
    if (*p) { *p = '\0'; p++; }

    // пропускаем пробелы перед данными
    while (*p == ' ') p++;
    if (*p == '\0') {
        out->data = NULL;
        return 1;
    }

    out->data = p;

    // данные до следующего пробела (или конца)
    while (*p && *p != ' ') p++;
    *p = '\0';

    return 1;
}

// Отправка ACK
static void send_ack(const char *cmd, const char *data)
{
    char buf[64];

    if (data)
        snprintf(buf, sizeof(buf), "\x02 ACK %s %s\r\n", cmd, data);
    else
        snprintf(buf, sizeof(buf), "\x02 ACK %s\r\n", cmd);

    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
}

void send_nak(void)
{
    char buf[] = "\x02 NAK\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
}

// Основной разбор команд
void parse_command(char *frame)
{
    CommandFrame cf;
    if (!split_frame(frame, &cf)) {
        send_nak();
        return;
    }

    // приводим команду к верхнему регистру
    for (char *p = cf.cmd; *p; ++p)
        *p = (char)toupper((unsigned char)*p);

    if (strcmp(cf.cmd, "START") == 0) {
        measuring_enabled = 1;
        smm_count = 0;
        smm_index = 0;
        send_ack("start", NULL);
    }
    else if (strcmp(cf.cmd, "STOP") == 0) {
        measuring_enabled = 0;
        send_ack("stop", NULL);
    }
    else if (strcmp(cf.cmd, "T_GET") == 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)T_ms);
        send_ack("T_get", buf);
    }
    else if (strcmp(cf.cmd, "T_SET") == 0) {
        if (!cf.data) { send_nak(); return; }
        uint32_t val = (uint32_t)atoi(cf.data);
        if (val < 1) val = 1;
        T_ms = val;

        char buf[16];
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)T_ms);
        send_ack("T_set", buf);
    }
    else if (strcmp(cf.cmd, "N_GET") == 0) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%u", smm_N);
        send_ack("N_get", buf);
    }
    else if (strcmp(cf.cmd, "N_SET") == 0) {
        if (!cf.data) { send_nak(); return; }
        uint32_t val = (uint32_t)atoi(cf.data);
        if (val < 1) val = 1;
        if (val > SMM_MAX_N) val = SMM_MAX_N;

        smm_N     = (uint8_t)val;
        smm_count = 0;
        smm_index = 0;

        char buf[4];
        snprintf(buf, sizeof(buf), "%u", smm_N);
        send_ack("N_set", buf);
    }
    else if (strcmp(cf.cmd, "ALPHA_GET") == 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%.3f", alpha);
        send_ack("alpha_get", buf);
    }
    else if (strcmp(cf.cmd, "ALPHA_SET") == 0) {
        if (!cf.data) { send_nak(); return; }
        float val = atof(cf.data);
        if (val < 0.0f) val = 0.0f;
        if (val > 1.0f) val = 1.0f;
        alpha = val;

        char buf[16];
        snprintf(buf, sizeof(buf), "%.3f", alpha);
        send_ack("alpha_set", buf);
    }
    else {
        send_nak();
    }
}

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
	  char dbg[32];
	      // печатаем код принятого байта
	      int len = sprintf(dbg, "[%d]\r\n", rx_byte);
	      HAL_UART_Transmit(&huart1, (uint8_t*)dbg, len, 100);
    if (rx_byte == '\r' || rx_byte == '\n')
    {
      if (rx_index > 0)
      {
        rx_buffer[rx_index] = '\0';           // закончить строку

        if (strcmp(rx_buffer, "START") == 0)  // ровно START
        {
          const char *resp = "OK: START\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t*)resp, strlen(resp), 100);
        }

        rx_index = 0;                         // готовим буфер к следующей команде
      }
    }
    else
    {
      if (rx_index < RX_BUFFER_SIZE - 1)      // защита от переполнения
      {
        rx_buffer[rx_index++] = (char)rx_byte;
      }
      else
      {
        rx_index = 0;                         // переполнен — сбросить
      }
    }

    // обязательно перезапустить приём
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
      if (!in_frame) {
          // ждём STX (0x02)
          if (rx_byte == 0x02) {
              in_frame = 1;
              rx_pos   = 0;
          }
      } else {
          // внутри кадра: ждём LF (0x0A)
          if (rx_byte == 0x0A) {
              // конец кадра
              rx_buf[rx_pos] = '\0';
              in_frame = 0;

              parse_command(rx_buf);   // разбор строки (без STX и LF)
          } else {
              if (rx_pos < UART_RX_BUF_SIZE - 1) {
                  rx_buf[rx_pos++] = (char)rx_byte;
              } else {
                  // переполнение - сброс
                  in_frame = 0;
                  rx_pos   = 0;
                  send_nak();
              }
          }
      }

      // перезапускаем приём следующего байта
      HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

/* USER CODE END 1 */
