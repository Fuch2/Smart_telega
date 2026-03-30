/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_02_Pin GPIO_PIN_14
#define SW_02_GPIO_Port GPIOC
#define SW_00_Pin GPIO_PIN_0
#define SW_00_GPIO_Port GPIOA
#define SW_01_Pin GPIO_PIN_1
#define SW_01_GPIO_Port GPIOA
#define SW_03_Pin GPIO_PIN_2
#define SW_03_GPIO_Port GPIOA
#define SW_04_Pin GPIO_PIN_3
#define SW_04_GPIO_Port GPIOA
#define SW_05_Pin GPIO_PIN_4
#define SW_05_GPIO_Port GPIOA
#define SW_06_Pin GPIO_PIN_5
#define SW_06_GPIO_Port GPIOA
#define SW_07_Pin GPIO_PIN_6
#define SW_07_GPIO_Port GPIOA
#define SW_08_Pin GPIO_PIN_7
#define SW_08_GPIO_Port GPIOA
#define SW_09_Pin GPIO_PIN_0
#define SW_09_GPIO_Port GPIOB
#define SW_10_Pin GPIO_PIN_1
#define SW_10_GPIO_Port GPIOB
#define SW_18_Pin GPIO_PIN_10
#define SW_18_GPIO_Port GPIOB
#define SW_19_Pin GPIO_PIN_11
#define SW_19_GPIO_Port GPIOB
#define SW_20_Pin GPIO_PIN_12
#define SW_20_GPIO_Port GPIOB
#define SW_21_Pin GPIO_PIN_13
#define SW_21_GPIO_Port GPIOB
#define SW_22_Pin GPIO_PIN_14
#define SW_22_GPIO_Port GPIOB
#define SW_23_Pin GPIO_PIN_15
#define SW_23_GPIO_Port GPIOB
#define SW_11_Pin GPIO_PIN_3
#define SW_11_GPIO_Port GPIOB
#define SW_12_Pin GPIO_PIN_4
#define SW_12_GPIO_Port GPIOB
#define SW_13_Pin GPIO_PIN_5
#define SW_13_GPIO_Port GPIOB
#define SW_14_Pin GPIO_PIN_6
#define SW_14_GPIO_Port GPIOB
#define SW_15_Pin GPIO_PIN_7
#define SW_15_GPIO_Port GPIOB
#define SW_16_Pin GPIO_PIN_8
#define SW_16_GPIO_Port GPIOB
#define SW_17_Pin GPIO_PIN_9
#define SW_17_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
