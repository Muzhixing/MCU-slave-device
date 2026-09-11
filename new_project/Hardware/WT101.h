/**
 ******************************************************************************
 * @file    WT101.h
 * @brief   HWT101 I2C yaw interface
 * @pin_resources PB6=I2C1_SCL, PB7=I2C1_SDA; external 4.7 kOhm pull-ups.
 * @peripherals I2C1 at 100 kHz.
 * @function Probes address 0x50 and reads signed yaw from register 0x3F.
 * @purpose Supplies current yaw without occupying a UART peripheral.
 * @migration Keeps the original raw/32768*180 yaw conversion.
 ******************************************************************************
 */
#ifndef WT101_H
#define WT101_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define WT101_I2C_ADDRESS            0x50U
#define WT101_YAW_REGISTER           0x3FU
#define WT101_I2C_DEFAULT_TIMEOUT_MS 10U

void WT101_Init(void);
HAL_StatusTypeDef WT101_IsReady(uint32_t timeout_ms);
HAL_StatusTypeDef WT101_ReadYaw(float *yaw_angle, uint32_t timeout_ms);

#endif
