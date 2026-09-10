/**
 ******************************************************************************
 * @file    WT101.c
 * @brief   HWT101 I2C yaw driver
 * @pin_resources PB6=I2C1_SCL, PB7=I2C1_SDA; external 4.7 kOhm pull-ups.
 * @peripherals I2C1 at 100 kHz.
 * @function Reads the two-byte signed yaw value from register 0x3F.
 * @purpose Supplies HWT101 yaw while preserving USART2 for Bluetooth.
 * @migration Source scale is unchanged: signed raw/32768*180 degrees.
 ******************************************************************************
 */
#include "WT101.h"

#include "I2C.h"

void WT101_Init(void)
{
    MX_I2C1_Init();
}

HAL_StatusTypeDef WT101_IsReady(uint32_t timeout_ms)
{
    return HAL_I2C_IsDeviceReady(&hi2c1,
                                 (uint16_t)(WT101_I2C_ADDRESS << 1U),
                                 2U,
                                 timeout_ms);
}

HAL_StatusTypeDef WT101_ReadYaw(float *yaw_angle, uint32_t timeout_ms)
{
    uint8_t yaw_data[2];
    int16_t yaw_raw;
    HAL_StatusTypeDef status;

    if (yaw_angle == NULL)
    {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(&hi2c1,
                              (uint16_t)(WT101_I2C_ADDRESS << 1U),
                              WT101_YAW_REGISTER,
                              I2C_MEMADD_SIZE_8BIT,
                              yaw_data,
                              2U,
                              timeout_ms);
    if (status != HAL_OK)
    {
        return status;
    }

    yaw_raw = (int16_t)(((uint16_t)yaw_data[1] << 8U) | yaw_data[0]);
    *yaw_angle = (float)yaw_raw / 32768.0f * 180.0f;
    return HAL_OK;
}
