#include "motor.h"

static TIM_HandleTypeDef pwm1, pwm9;
static uint8_t ready;
static int8_t last_sign[MOTOR_COUNT];
static uint32_t stopped_at[MOTOR_COUNT];
static uint8_t running[MOTOR_COUNT];
typedef struct {
  GPIO_TypeDef *in1_port;
  uint16_t in1;
  GPIO_TypeDef *in2_port;
  uint16_t in2;
  TIM_HandleTypeDef *timer;
  uint32_t channel;
} MotorPins;
static const MotorPins pins[MOTOR_COUNT] = {
  {GPIOB, GPIO_PIN_15, GPIOD, GPIO_PIN_1, &pwm9, TIM_CHANNEL_2},
  {GPIOD, GPIO_PIN_2, GPIOD, GPIO_PIN_3, &pwm1, TIM_CHANNEL_2},
  {GPIOD, GPIO_PIN_4, GPIOD, GPIO_PIN_5, &pwm1, TIM_CHANNEL_3},
  {GPIOD, GPIO_PIN_6, GPIOD, GPIO_PIN_7, &pwm1, TIM_CHANNEL_4}
};

void Motor_StopAll(void)
{
  uint32_t i;
  if (!ready) { return; }
  for (i = 0; i < MOTOR_COUNT; ++i) {
    __HAL_TIM_SET_COMPARE(pins[i].timer, pins[i].channel, 0);
    HAL_GPIO_WritePin(pins[i].in1_port, pins[i].in1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(pins[i].in2_port, pins[i].in2, GPIO_PIN_RESET);
    if (running[i]) { stopped_at[i] = HAL_GetTick(); }
    running[i] = 0;
  }
}

HAL_StatusTypeDef Motor_Set(MotorId id, int32_t duty)
{
  const MotorPins *p;
  int8_t sign;
  if (!ready || (uint32_t)id >= MOTOR_COUNT) { return HAL_ERROR; }
  p = &pins[id];
  if (duty > 999) { duty = 999; }
  if (duty < -999) { duty = -999; }
  /* Right-side motors C/D are mounted with opposite polarity to A/B. */
  if (id == MOTOR_RF || id == MOTOR_RB) { duty = -duty; }
  sign = duty > 0 ? 1 : (duty < 0 ? -1 : 0);
  if (sign && last_sign[id] && sign != last_sign[id] &&
      (running[id] || (uint32_t)(HAL_GetTick() - stopped_at[id]) < 500U)) {
    return HAL_BUSY;
  }
  __HAL_TIM_SET_COMPARE(p->timer, p->channel, 0);
  HAL_GPIO_WritePin(p->in1_port, p->in1, sign > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(p->in2_port, p->in2, sign < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(p->timer, p->channel, duty < 0 ? -duty : duty);
  if (!sign && running[id]) { stopped_at[id] = HAL_GetTick(); }
  running[id] = sign != 0;
  if (sign) { last_sign[id] = sign; }
  return HAL_OK;
}

static HAL_StatusTypeDef InitTimer(TIM_HandleTypeDef *h, TIM_TypeDef *instance)
{
  h->Instance = instance;
  h->Init.Prescaler = 15;
  h->Init.CounterMode = TIM_COUNTERMODE_UP;
  h->Init.Period = 999;
  h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  h->Init.RepetitionCounter = 0;
  h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  return HAL_TIM_PWM_Init(h);
}

HAL_StatusTypeDef Motor_Init(void)
{
  GPIO_InitTypeDef g = {0};
  TIM_OC_InitTypeDef oc = {0};
  uint32_t i;
  if (ready) { Motor_StopAll(); return HAL_OK; }
  /* Both PWM timers reside on APB2. Reject an unexpected clock tree. */
  if (HAL_RCC_GetHCLKFreq() != 168000000U ||
      HAL_RCC_GetPCLK2Freq() != 84000000U) { return HAL_ERROR; }
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
      GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_11 |
      GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = GPIO_PIN_15; HAL_GPIO_Init(GPIOB, &g);
  g.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
      GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7; HAL_GPIO_Init(GPIOD, &g);
  /* PE1 pump off; hold PWM pins low until timers have zero compare values. */
  g.Pin = GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  HAL_GPIO_Init(GPIOE, &g);
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM9_CLK_ENABLE();
  if (InitTimer(&pwm1, TIM1) != HAL_OK || InitTimer(&pwm9, TIM9) != HAL_OK) {
    return HAL_ERROR;
  }
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  for (i = 0; i < MOTOR_COUNT; ++i) {
    if (HAL_TIM_PWM_ConfigChannel(pins[i].timer, &oc, pins[i].channel) != HAL_OK) {
      return HAL_ERROR;
    }
  }
  ready = 1;
  for (i = 0; i < MOTOR_COUNT; ++i) {
    if (HAL_TIM_PWM_Start(pins[i].timer, pins[i].channel) != HAL_OK) {
      Motor_StopAll(); return HAL_ERROR;
    }
  }
  g.Mode = GPIO_MODE_AF_PP;
  g.Alternate = GPIO_AF1_TIM1;
  g.Pin = GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14; HAL_GPIO_Init(GPIOE, &g);
  g.Alternate = GPIO_AF3_TIM9;
  g.Pin = GPIO_PIN_6; HAL_GPIO_Init(GPIOE, &g);
  Motor_StopAll();
  return HAL_OK;
}
