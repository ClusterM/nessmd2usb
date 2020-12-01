/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
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
typedef struct
{
  uint8_t report_id;
  int8_t x, y;
  uint8_t buttons;
} Gamepad_Report_t;

typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;
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
#define NES_LATCH_Pin GPIO_PIN_0
#define NES_LATCH_GPIO_Port GPIOA
#define NES_CLOCK_Pin GPIO_PIN_1
#define NES_CLOCK_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOA
#define NES1_SENSOR_Pin GPIO_PIN_4
#define NES1_SENSOR_GPIO_Port GPIOA
#define NES1_TRIGGER_Pin GPIO_PIN_5
#define NES1_TRIGGER_GPIO_Port GPIOA
#define NES2_SENSOR_Pin GPIO_PIN_6
#define NES2_SENSOR_GPIO_Port GPIOA
#define NES2_TRIGGER_Pin GPIO_PIN_7
#define NES2_TRIGGER_GPIO_Port GPIOA
#define SMD_SELECT_Pin GPIO_PIN_0
#define SMD_SELECT_GPIO_Port GPIOB
#define SMD1_D0_Pin GPIO_PIN_2
#define SMD1_D0_GPIO_Port GPIOB
#define SMD2_D1_Pin GPIO_PIN_10
#define SMD2_D1_GPIO_Port GPIOB
#define SMD2_D2_Pin GPIO_PIN_11
#define SMD2_D2_GPIO_Port GPIOB
#define SMD2_D3_Pin GPIO_PIN_12
#define SMD2_D3_GPIO_Port GPIOB
#define SMD2_D4_Pin GPIO_PIN_13
#define SMD2_D4_GPIO_Port GPIOB
#define SMD2_D5_Pin GPIO_PIN_14
#define SMD2_D5_GPIO_Port GPIOB
#define NES1_DATA_Pin GPIO_PIN_8
#define NES1_DATA_GPIO_Port GPIOA
#define NES2_DATA_Pin GPIO_PIN_9
#define NES2_DATA_GPIO_Port GPIOA
#define SMD1_D1_Pin GPIO_PIN_3
#define SMD1_D1_GPIO_Port GPIOB
#define SMD1_D2_Pin GPIO_PIN_4
#define SMD1_D2_GPIO_Port GPIOB
#define SMD1_D3_Pin GPIO_PIN_6
#define SMD1_D3_GPIO_Port GPIOB
#define SMD1_D4_Pin GPIO_PIN_7
#define SMD1_D4_GPIO_Port GPIOB
#define SMD1_D5_Pin GPIO_PIN_8
#define SMD1_D5_GPIO_Port GPIOB
#define SMD2_D0_Pin GPIO_PIN_9
#define SMD2_D0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
