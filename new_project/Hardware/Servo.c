/**
 ******************************************************************************
 * @file    Servo.c
 * @brief   Two-channel 50 Hz servo PWM driver migrated from the original project
 *
 * @pin_resources
 *   - PA6: TIM3_CH1, original servo 1 output.
 *   - PA7: TIM3_CH2, original horizontal servo 2 output.
 *
 * @migration
 *   - Keeps the original 500..2500 us pulse range.
 *   - Keeps servo 1 at 0..270 degrees and servo 2 at 0..180 degrees.
 ******************************************************************************
 */

#include "Servo.h"

#define SERVO_MIN_PULSE_US 500U
#define SERVO_MAX_PULSE_US 2500U

TIM_HandleTypeDef htim3;

static void Servo_FailStop(void)
{
    while (1)
    {
    }
}

void Servo_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_MasterConfigTypeDef master = {0};
    TIM_OC_InitTypeDef channel = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 19999U;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Servo_FailStop();
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &master) != HAL_OK)
    {
        Servo_FailStop();
    }

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = SERVO_MIN_PULSE_US;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if ((HAL_TIM_PWM_ConfigChannel(&htim3, &channel, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_ConfigChannel(&htim3, &channel, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK))
    {
        Servo_FailStop();
    }
}

void Servo_SetAngle_1(uint8_t angle)
{
    uint32_t pulse_width = SERVO_MIN_PULSE_US +
        ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 270U;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_width);
}

void Servo_SetAngle_2(uint8_t angle)
{
    uint32_t pulse_width;

    if (angle > 180U)
    {
        angle = 180U;
    }
    pulse_width = SERVO_MIN_PULSE_US +
        ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180U;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_width);
}
