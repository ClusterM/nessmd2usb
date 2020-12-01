#include "main.h"
#include "gamepad.h"

inline static void _delay_us(uint32_t t)
{
  TIM1->CNT = 0;
  while (TIM1->CNT < t)
  {
  }
}

static uint32_t get_nes_gamepad(void)
{
  uint32_t gamepad_data = 0;
  HAL_GPIO_WritePin(NES_LATCH_GPIO_Port, NES_LATCH_Pin, GPIO_PIN_SET); // Latch
  _delay_us(1);
  HAL_GPIO_WritePin(NES_LATCH_GPIO_Port, NES_LATCH_Pin, GPIO_PIN_RESET); // Latch
  _delay_us(1);
  int b;
  for (b = 0; b < 8; b++)
  {
    HAL_GPIO_WritePin(NES_CLOCK_GPIO_Port, NES_CLOCK_Pin, GPIO_PIN_RESET); // Clock
    _delay_us(1);
    gamepad_data |= (uint32_t) HAL_GPIO_ReadPin(NES1_DATA_GPIO_Port, NES1_DATA_Pin) << b;
    gamepad_data |= (uint32_t) ((HAL_GPIO_ReadPin(NES2_DATA_GPIO_Port, NES2_DATA_Pin)) << b) << 8;
    HAL_GPIO_WritePin(NES_CLOCK_GPIO_Port, NES_CLOCK_Pin, GPIO_PIN_SET); // Clock
    _delay_us(1);
  }
  return gamepad_data;
}

uint32_t get_nes_gamepad_decoded(void)
{
  return ~get_nes_gamepad();
}

static void init_smd_gamepad(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SMD_SELECT_GPIO_Port, SMD_SELECT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SMD_SELECT_Pin */
  GPIO_InitStruct.Pin = SMD_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SMD_SELECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMD1_D0_Pin SMD2_D1_Pin SMD2_D2_Pin SMD2_D3_Pin
   SMD2_D4_Pin SMD2_D5_Pin SMD1_D1_Pin SMD1_D2_Pin
   SMD1_D3_Pin SMD1_D4_Pin SMD1_D5_Pin SMD2_D0_Pin */
  GPIO_InitStruct.Pin = SMD1_D0_Pin | SMD2_D1_Pin | SMD2_D2_Pin | SMD2_D3_Pin | SMD2_D4_Pin | SMD2_D5_Pin | SMD1_D1_Pin | SMD1_D2_Pin | SMD1_D3_Pin
      | SMD1_D4_Pin | SMD1_D5_Pin | SMD2_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void init_dendy_9pin(uint8_t n)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  if (!n)
  {
    // Data 1 aka data, input
    GPIO_InitStruct.Pin = SMD1_D1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Data 2 aka latch, output
    HAL_GPIO_WritePin(GPIOB, SMD1_D2_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SMD1_D2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Data 3 aka clock, output
    HAL_GPIO_WritePin(GPIOB, SMD1_D3_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SMD1_D3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Data 4 aka VCC, output
    HAL_GPIO_WritePin(GPIOB, SMD1_D4_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SMD1_D4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  } else
  {
    // Data 1 aka data, input
    GPIO_InitStruct.Pin = SMD2_D1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Data 2 aka latch, output
    HAL_GPIO_WritePin(GPIOB, SMD2_D2_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SMD2_D2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Data 3 aka clock, output
    HAL_GPIO_WritePin(GPIOB, SMD2_D3_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SMD2_D3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Data 4 aka VCC, output
    HAL_GPIO_WritePin(GPIOB, SMD2_D4_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SMD2_D4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

static uint32_t get_smd_gamepad()
{
  uint8_t gamepad_data_low = 0xFF;
  uint8_t gamepad_data_high = 0xFF;
  uint8_t gamepad_data_low2 = 0xFF;
  uint8_t gamepad_data_high2 = 0xFF;
  HAL_GPIO_WritePin(SMD_SELECT_GPIO_Port, SMD_SELECT_Pin, GPIO_PIN_RESET); // Select - low
  _delay_us(1);
  gamepad_data_low = HAL_GPIO_ReadPin(SMD1_D0_GPIO_Port, SMD1_D0_Pin) | (HAL_GPIO_ReadPin(SMD1_D1_GPIO_Port, SMD1_D1_Pin) << 1)
      | (HAL_GPIO_ReadPin(SMD1_D2_GPIO_Port, SMD1_D2_Pin) << 2) | (HAL_GPIO_ReadPin(SMD1_D3_GPIO_Port, SMD1_D3_Pin) << 3)
      | (HAL_GPIO_ReadPin(SMD1_D4_GPIO_Port, SMD1_D4_Pin) << 4) | (HAL_GPIO_ReadPin(SMD1_D5_GPIO_Port, SMD1_D5_Pin) << 5);
  gamepad_data_low2 = HAL_GPIO_ReadPin(SMD2_D0_GPIO_Port, SMD2_D0_Pin) | (HAL_GPIO_ReadPin(SMD2_D1_GPIO_Port, SMD2_D1_Pin) << 1)
      | (HAL_GPIO_ReadPin(SMD2_D2_GPIO_Port, SMD2_D2_Pin) << 2) | (HAL_GPIO_ReadPin(SMD2_D3_GPIO_Port, SMD2_D3_Pin) << 3)
      | (HAL_GPIO_ReadPin(SMD2_D4_GPIO_Port, SMD2_D4_Pin) << 4) | (HAL_GPIO_ReadPin(SMD2_D5_GPIO_Port, SMD2_D5_Pin) << 5);
  HAL_GPIO_WritePin(SMD_SELECT_GPIO_Port, SMD_SELECT_Pin, GPIO_PIN_SET); // Select - higt
  _delay_us(1);
  gamepad_data_high = HAL_GPIO_ReadPin(SMD1_D0_GPIO_Port, SMD1_D0_Pin) | (HAL_GPIO_ReadPin(SMD1_D1_GPIO_Port, SMD1_D1_Pin) << 1)
      | (HAL_GPIO_ReadPin(SMD1_D2_GPIO_Port, SMD1_D2_Pin) << 2) | (HAL_GPIO_ReadPin(SMD1_D3_GPIO_Port, SMD1_D3_Pin) << 3)
      | (HAL_GPIO_ReadPin(SMD1_D4_GPIO_Port, SMD1_D4_Pin) << 4) | (HAL_GPIO_ReadPin(SMD1_D5_GPIO_Port, SMD1_D5_Pin) << 5);
  gamepad_data_high2 = HAL_GPIO_ReadPin(SMD2_D0_GPIO_Port, SMD2_D0_Pin) | (HAL_GPIO_ReadPin(SMD2_D1_GPIO_Port, SMD2_D1_Pin) << 1)
      | (HAL_GPIO_ReadPin(SMD2_D2_GPIO_Port, SMD2_D2_Pin) << 2) | (HAL_GPIO_ReadPin(SMD2_D3_GPIO_Port, SMD2_D3_Pin) << 3)
      | (HAL_GPIO_ReadPin(SMD2_D4_GPIO_Port, SMD2_D4_Pin) << 4) | (HAL_GPIO_ReadPin(SMD2_D5_GPIO_Port, SMD2_D5_Pin) << 5);
  return ((uint32_t) gamepad_data_high2 << 24) | ((uint32_t) gamepad_data_low2 << 16) | ((uint32_t) gamepad_data_high << 8) | gamepad_data_low;
}

static uint8_t get_dendy_9pin(uint8_t n)
{
  uint8_t gamepad_data = 0;
  if (!n)
  {
    // Data 2 aka latch, low
    HAL_GPIO_WritePin(SMD1_D2_GPIO_Port, SMD1_D2_Pin, GPIO_PIN_SET);
    _delay_us(1);
    HAL_GPIO_WritePin(SMD1_D2_GPIO_Port, SMD1_D2_Pin, GPIO_PIN_RESET);
    int b;
    for (b = 0; b < 8; b++)
    {
      HAL_GPIO_WritePin(SMD1_D3_GPIO_Port, SMD1_D3_Pin, GPIO_PIN_RESET); // Data 3 aka clock, low
      _delay_us(1);
      gamepad_data |= HAL_GPIO_ReadPin(SMD1_D1_GPIO_Port, SMD1_D1_Pin) << b;
      HAL_GPIO_WritePin(SMD1_D3_GPIO_Port, SMD1_D3_Pin, GPIO_PIN_SET); // Data 3 aka clock, hi
      _delay_us(1);
    }
  } else
  {
    HAL_GPIO_WritePin(SMD2_D2_GPIO_Port, SMD2_D2_Pin, GPIO_PIN_SET);
    _delay_us(1);
    HAL_GPIO_WritePin(SMD2_D2_GPIO_Port, SMD2_D2_Pin, GPIO_PIN_RESET);
    int b;
    for (b = 0; b < 8; b++)
    {
      HAL_GPIO_WritePin(SMD2_D3_GPIO_Port, SMD2_D3_Pin, GPIO_PIN_RESET); // Data 3 aka clock, low
      _delay_us(1);
      gamepad_data |= HAL_GPIO_ReadPin(SMD2_D1_GPIO_Port, SMD2_D1_Pin) << b;
      HAL_GPIO_WritePin(SMD2_D3_GPIO_Port, SMD2_D3_Pin, GPIO_PIN_SET); // Data 3 aka clock, hi
      _delay_us(1);
    }
  }
  return gamepad_data;
}

uint32_t get_smd_gamepad_decoded(void)
{
  uint32_t result = 0;
  uint8_t smd_detected[2] = { 0, 0 };
  uint8_t b, c, d;
  for (c = 0; c < 4; c++)
  {
    uint32_t smd_gamepad_data = get_smd_gamepad();
    for (d = 0; d < 2; d++)
    { // for each controller
      if ((smd_gamepad_data & 0b00001111) || (c < 2)) // 3-button mode
      {
        for (b = 0; b <= 13; b++)
        {
          if (!((smd_gamepad_data >> b) & 1))
          {
            switch (b)
            {
            case 0: // Up
              set_bit(result, 8 + d * 16);
              break;
            case 1: // Down
              set_bit(result, 9 + d * 16);
              break;
            case 2: // always low
            case 3:
              smd_detected[d] = 1;
              break;
            case 4: // A
              set_bit(result, 0 + d * 16);
              break;
            case 5: // Start
              set_bit(result, 6 + d * 16);
              break;
            case 10: // Left
              set_bit(result, 10 + d * 16);
              break;
            case 11: // Right
              set_bit(result, 11 + d * 16);
              break;
            case 12: // B
              set_bit(result, 1 + d * 16);
              break;
            case 13: // C
              set_bit(result, 2 + d * 16);
              break;
            }
          }
        }
      } else
      { // 6-button mode
        for (b = 4; b <= 11; b++)
        {
          if (!((smd_gamepad_data >> b) & 1))
          {
            switch (b)
            {
            case 4: // A
              set_bit(result, 0 + d * 16);
              break;
            case 5: // Start
              set_bit(result, 6 + d * 16);
              break;
            case 8: // Z
              set_bit(result, 5 + d * 16);
              break;
            case 9: // Y
              set_bit(result, 4 + d * 16);
              break;
            case 10: // X
              set_bit(result, 3 + d * 16);
              break;
            case 11: // Mode
              set_bit(result, 7 + d * 16);
              break;
            }
          }
        }
      }
      smd_gamepad_data >>= 16;
    }
  }
  if (!smd_detected[0] || !smd_detected[1])
  { // SMD gamepad is not connected?
    // so maybe it's 9-pin dendy gamepad?
    for (d = 0; d < 2; d++)
    {
      if (!smd_detected[d])
      {
        init_dendy_9pin(d);
        _delay_us(10);
        uint32_t dendy_data = ~get_dendy_9pin(d);
        result &= ~(0xFFFFUL << (d * 16));
        result |= ((dendy_data & 0x0F) | ((dendy_data & 0xF0) << 4)) << (16 * d);
      }
    }
    init_smd_gamepad(); // back to SMD mode
  }

  return result;
}
