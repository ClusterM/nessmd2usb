/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include "gamepad.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define USE_LIGHT_GUN
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch3;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t pwm_values[8 * 3 + 1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void update_led()
{
  //HAL_Delay(1);
  htim2.Instance->CR1 &= ~TIM_CR1_CEN;
  htim2.Instance->CCR3 = 0;
  htim2.Instance->CNT = 0;
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (void*) pwm_values, sizeof(pwm_values));
}

static void set_led_color(uint8_t r, uint8_t g, uint8_t b)
{
//  r /= 4;
//  g /= 4;
//  b /= 4;
  uint8_t i;
  const uint8_t led = 0;
  for (i = 0; i < 8; i++)
    pwm_values[led * 8 * 3 + i] = ((r >> (7 - i)) & 1) ? 67 : 22;
  for (i = 0; i < 8; i++)
    pwm_values[led * 8 * 3 + 8 + i] = ((g >> (7 - i)) & 1) ? 67 : 22;
  for (i = 0; i < 8; i++)
    pwm_values[led * 8 * 3 + 16 + i] = ((b >> (7 - i)) & 1) ? 67 : 22;
  update_led();
}

static rgb hsv2rgb(hsv in)
{
  double hh, p, q, t, ff;
  long i;
  rgb out;
  if (in.s <= 0.0)
  {       // < is bogus, just shuts up warnings
    out.r = in.v;
    out.g = in.v;
    out.b = in.v;
    return out;
  }
  hh = in.h;
  while (hh > 360)
    hh -= 360;
  hh /= 60.0;
  i = (long) hh;
  ff = hh - i;
  p = in.v * (1.0 - in.s);
  q = in.v * (1.0 - (in.s * ff));
  t = in.v * (1.0 - (in.s * (1.0 - ff)));

  switch (i)
  {
  case 0:
    out.r = in.v;
    out.g = t;
    out.b = p;
    break;
  case 1:
    out.r = q;
    out.g = in.v;
    out.b = p;
    break;
  case 2:
    out.r = p;
    out.g = in.v;
    out.b = t;
    break;

  case 3:
    out.r = p;
    out.g = q;
    out.b = in.v;
    break;
  case 4:
    out.r = t;
    out.g = p;
    out.b = in.v;
    break;
  case 5:
  default:
    out.r = in.v;
    out.g = p;
    out.b = q;
    break;
  }
  return out;
}

static void set_axes(uint8_t dpad, Gamepad_Report_t *joystick_data)
{
  if (dpad & 1)
    joystick_data->y = -127;
  else if (dpad & 2)
    joystick_data->y = 127;
  else
    joystick_data->y = 0;
  if (dpad & 4)
    joystick_data->x = -127;
  else if (dpad & 8)
    joystick_data->x = 127;
  else
    joystick_data->x = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Gamepad_Report_t joystick_data;
  uint8_t prev_trigger1 = 1;
  uint8_t prev_sensor1 = 1;
  uint8_t prev_trigger2 = 1;
  uint8_t prev_sensor2 = 1;

  while (1)
  {
    switch (hUsbDeviceFS.dev_state)
    {
    case USBD_STATE_CONFIGURED:
      break;
    case USBD_STATE_ADDRESSED:
      set_led_color(0xFF, 0xFF, 0x00);
      HAL_Delay(1);
      continue;
    default:
      set_led_color(0xFF, 0x00, 0x00);
      HAL_Delay(1);
      continue;
    }

    uint32_t smd_gamepad_data = get_smd_gamepad_decoded();
    uint16_t nes_gamepad_data = get_nes_gamepad_decoded();
#ifdef USE_LIGHT_GUN
    uint8_t trigger1 = HAL_GPIO_ReadPin(NES1_TRIGGER_GPIO_Port, NES1_TRIGGER_Pin);
    uint8_t sensor1 = HAL_GPIO_ReadPin(NES1_SENSOR_GPIO_Port, NES1_SENSOR_Pin);
    uint8_t trigger2 = HAL_GPIO_ReadPin(NES2_TRIGGER_GPIO_Port, NES2_TRIGGER_Pin);
    uint8_t sensor2 = HAL_GPIO_ReadPin(NES2_SENSOR_GPIO_Port, NES2_SENSOR_Pin);
#else
    const uint8_t trigger1 = 0;
    const uint8_t sensor1 = 0;
    const uint8_t trigger2 = 0;
    const uint8_t sensor2 = 0;
#endif

    TIM1->CNT = 0;

    joystick_data.report_id = 1;
    joystick_data.buttons = (nes_gamepad_data & 0x0F) | (trigger1 << 4) | (sensor1 << 5);
    set_axes((nes_gamepad_data >> 4) & 0x0F, &joystick_data);
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (void*) &joystick_data, sizeof(joystick_data));
    while (((USBD_CUSTOM_HID_HandleTypeDef*) hUsbDeviceFS.pClassData)->state != CUSTOM_HID_IDLE)
    {
      if (TIM1->CNT > 50000)
        break;
    }

    joystick_data.report_id = 2;
    joystick_data.buttons = smd_gamepad_data & 0xFF;
    set_axes((smd_gamepad_data >> 8) & 0x0F, &joystick_data);
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (void*) &joystick_data, sizeof(joystick_data));
    while (((USBD_CUSTOM_HID_HandleTypeDef*) hUsbDeviceFS.pClassData)->state != CUSTOM_HID_IDLE)
    {
      if (TIM1->CNT > 50000)
        break;
    }

    joystick_data.report_id = 3;
    joystick_data.buttons = ((nes_gamepad_data >> 8) & 0x0F) | (trigger2 << 4) | (sensor2 << 5);
    set_axes((nes_gamepad_data >> 12) & 0x0F, &joystick_data);
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (void*) &joystick_data, sizeof(joystick_data));
    while (((USBD_CUSTOM_HID_HandleTypeDef*) hUsbDeviceFS.pClassData)->state != CUSTOM_HID_IDLE)
    {
      if (TIM1->CNT > 50000)
        break;
    }

    joystick_data.report_id = 4;
    joystick_data.buttons = (smd_gamepad_data >> 16) & 0xFF;
    set_axes((smd_gamepad_data >> 24) & 0x0F, &joystick_data);
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (void*) &joystick_data, sizeof(joystick_data));
    while (((USBD_CUSTOM_HID_HandleTypeDef*) hUsbDeviceFS.pClassData)->state != CUSTOM_HID_IDLE)
    {
      if (TIM1->CNT > 50000)
        break;
    }

    if ((sensor1 && !prev_sensor1) || (sensor2 && !prev_sensor2))
    {
      set_led_color(0xFF, 0xFF, 0xFF);
    } else if ((trigger1 && !prev_trigger1) || (trigger2 && !prev_trigger2))
    {
      set_led_color(0x40, 0x40, 0);
    } else
    {
      uint8_t button_count = 0;
      uint8_t i;
      for (i = 0; i < 16; i++)
        if ((nes_gamepad_data >> i) & 1)
          button_count++;
      for (i = 0; i < 32; i++)
        if ((smd_gamepad_data >> i) & 1)
          button_count++;
      hsv v;
      v.h = 100 + button_count * 15;
      v.s = 1;
      v.v = 0.1 + 0.05 * button_count;
      if (v.v > 1)
        v.v = 1;
      rgb r = hsv2rgb(v);
      if (r.r > 1)
        r.r = 1;
      if (r.g > 1)
        r.g = 1;
      if (r.b > 1)
        r.b = 1;
      set_led_color(r.r * 0xFF, r.g * 0xFF, r.b * 0xFF);
    }

    prev_trigger1 = trigger1;
    prev_sensor1 = sensor1;
    prev_trigger2 = trigger2;
    prev_sensor2 = sensor2;

    // 1ms delay
    while (TIM1->CNT < 1000)
    {
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 89;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NES_LATCH_Pin|NES_CLOCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SMD_SELECT_GPIO_Port, SMD_SELECT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NES_LATCH_Pin NES_CLOCK_Pin */
  GPIO_InitStruct.Pin = NES_LATCH_Pin|NES_CLOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NES1_SENSOR_Pin NES2_SENSOR_Pin */
  GPIO_InitStruct.Pin = NES1_SENSOR_Pin|NES2_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NES1_TRIGGER_Pin NES2_TRIGGER_Pin NES1_DATA_Pin NES2_DATA_Pin */
  GPIO_InitStruct.Pin = NES1_TRIGGER_Pin|NES2_TRIGGER_Pin|NES1_DATA_Pin|NES2_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMD_SELECT_Pin */
  GPIO_InitStruct.Pin = SMD_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SMD_SELECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMD1_D0_Pin SMD2_D1_Pin SMD2_D2_Pin SMD2_D3_Pin
                           SMD2_D4_Pin SMD2_D5_Pin SMD1_D1_Pin SMD1_D2_Pin
                           SMD1_D3_Pin SMD1_D4_Pin SMD1_D5_Pin SMD2_D0_Pin */
  GPIO_InitStruct.Pin = SMD1_D0_Pin|SMD2_D1_Pin|SMD2_D2_Pin|SMD2_D3_Pin
                          |SMD2_D4_Pin|SMD2_D5_Pin|SMD1_D1_Pin|SMD1_D2_Pin
                          |SMD1_D3_Pin|SMD1_D4_Pin|SMD1_D5_Pin|SMD2_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
