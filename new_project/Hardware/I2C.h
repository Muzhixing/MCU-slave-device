/**
 ******************************************************************************
 * @file    I2C.h
 * @brief   I2C1 interface for the HWT101 sensor bus
 * @pin_resources PB6=SCL, PB7=SDA; @peripherals I2C1
 * @function Exposes the 100 kHz I2C1 initialization and handle.
 * @purpose Provides the production HWT101 register transport.
 * @migration Source: src/Core/Inc/i2c.h; adapted from PB8/PB9 to PB6/PB7.
 ******************************************************************************
 */
#ifndef I2C_H
#define I2C_H
#include "stm32f1xx_hal.h"
extern I2C_HandleTypeDef hi2c1;
void MX_I2C1_Init(void);
#endif
