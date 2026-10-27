#include "MotorEncoder.h"

static TIM_HandleTypeDef enc[4];
static uint16_t previous[4];
static const int8_t signs[4] = {MOTOR_ENCODER_SIGN_A, MOTOR_ENCODER_SIGN_B,
  MOTOR_ENCODER_SIGN_C, MOTOR_ENCODER_SIGN_D};

uint16_t MotorEncoder_ReadRaw(MotorId id)
{
  return (uint32_t)id < 4U ? (uint16_t)__HAL_TIM_GET_COUNTER(&enc[id]) : 0;
}

HAL_StatusTypeDef MotorEncoder_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  TIM_Encoder_InitTypeDef cfg = {0};
  TIM_TypeDef *timers[4] = {TIM2, TIM3, TIM4, TIM8};
  uint32_t i;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1; gpio.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7; gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13; gpio.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &gpio);
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7; gpio.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &gpio);
  cfg.EncoderMode = TIM_ENCODERMODE_TI12;
  cfg.IC1Polarity = TIM_ICPOLARITY_RISING;
  cfg.IC2Polarity = TIM_ICPOLARITY_RISING;
  cfg.IC1Selection = cfg.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  cfg.IC1Prescaler = cfg.IC2Prescaler = TIM_ICPSC_DIV1;
  cfg.IC1Filter = cfg.IC2Filter = 6;
  for (i=0; i<4; ++i) {
    enc[i].Instance = timers[i];
    enc[i].Init.Prescaler = 0;
    enc[i].Init.CounterMode = TIM_COUNTERMODE_UP;
    enc[i].Init.Period = 65535;
    enc[i].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    enc[i].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Encoder_Init(&enc[i], &cfg) != HAL_OK) { return HAL_ERROR; }
    __HAL_TIM_SET_COUNTER(&enc[i], 0);
    previous[i] = 0;
    if (HAL_TIM_Encoder_Start(&enc[i], TIM_CHANNEL_ALL) != HAL_OK) { return HAL_ERROR; }
  }
  return HAL_OK;
}

void MotorEncoder_Sample(uint32_t elapsed_ms, int32_t pulses[4], int32_t speeds[4])
{
  uint32_t i;
  for (i=0; i<4; ++i) {
    uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(&enc[i]);
    pulses[i] = MotorEncoder_Delta(previous[i], current) * signs[i];
    previous[i] = current;
    speeds[i] = MotorEncoder_PulseToSpeed(pulses[i], elapsed_ms);
  }
}
