/**
 ******************************************************************************
 * @file    Servo.h
 * @brief   Original two-channel servo PWM interface
 *
 * @pin_resources
 *   - Servo 1: PA6, TIM3_CH1.
 *   - Servo 2 (horizontal): PA7, TIM3_CH2.
 ******************************************************************************
 */

#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim3;

void Servo_Init(void);
void Servo_SetAngle_1(uint8_t angle);
void Servo_SetAngle_2(uint8_t angle);

#endif
