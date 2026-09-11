/**
 ******************************************************************************
 * @file    State_Machine.c
 * @brief   Formal integration of Bluetooth, HWT101, chassis, servo and CAN axes
 *
 * @pin_resources
 *   - HC-08 USART2: PA2/PA3.
 *   - HWT101 I2C1: PB6/PB7.
 *   - Horizontal servo: PA7/TIM3_CH2.
 *   - CAN1: PA11/PA12 through the external transceiver.
 *   - Motor pins are owned by Motor.c.
 ******************************************************************************
 */

#include "State_Machine.h"

#include "Emm_V5.h"
#include "Motor.h"
#include "Servo.h"
#include "UART.h"
#include "UpperComputer.h"
#include "WT101.h"
#include "can.h"
#include "hcan.h"

#define STATE_HWT101_PERIOD_MS 10U
#define STATE_STEEL_ADDR       1U
#define STATE_CROSSBAR_ADDR    2U

static System_State_t current_state;
static uint8_t control_packet_received;
static uint32_t last_yaw_tick;

static void State_EnterError(void)
{
    motor_stop_all();
    current_state = STATE_ERROR;
}

static uint8_t State_CanStarted(void)
{
    return (uint8_t)((can_filter_status == HAL_OK) &&
                     (can_notify_status == HAL_OK) &&
                     (can_start_status == HAL_OK));
}

static uint8_t State_CanCommandSent(void)
{
    return (uint8_t)(can_tx_status == HAL_OK);
}

static void State_InitHardware(void)
{
    UART_Init(115200U);
    UART_Enable_Receive();

    Motor_Init();
    motor_stop_all();

    WT101_Init();
    if (WT101_IsReady(WT101_I2C_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        State_EnterError();
        return;
    }

    Servo_Init();
    Servo_SetAngle_2(90U);

    MX_CAN_Init();
    CAN_Start(CAN_NUM);
    if (State_CanStarted() == 0U)
    {
        State_EnterError();
        return;
    }

    HAL_Delay(1000U);
    Emm_V5_Origin_Modify_Params(STATE_STEEL_ADDR, false, 2U, 0U,
                                30U, 20000U, 30U, 800U, 60U, false);
    if (State_CanCommandSent() == 0U)
    {
        State_EnterError();
        return;
    }
    HAL_Delay(100U);

    Emm_V5_En_Control(STATE_STEEL_ADDR, true, false);
    if (State_CanCommandSent() == 0U)
    {
        State_EnterError();
        return;
    }
    HAL_Delay(100U);

    Emm_V5_Origin_Trigger_Return(STATE_STEEL_ADDR, 2U, false);
    if (State_CanCommandSent() == 0U)
    {
        State_EnterError();
        return;
    }
    HAL_Delay(30000U);

    Emm_V5_En_Control(STATE_CROSSBAR_ADDR, true, false);
    if (State_CanCommandSent() == 0U)
    {
        State_EnterError();
        return;
    }
    HAL_Delay(100U);

    motor_vx = 0U;
    motor_vy = 0U;
    target_yaw = 0.0f;
    Motor_ResetHeadingControl();
    motor_stop_all();
    control_packet_received = 0U;
    last_yaw_tick = HAL_GetTick() - STATE_HWT101_PERIOD_MS;
    current_state = STATE_IDLE;
}

static void State_RunIdle(void)
{
    uint32_t now;
    float current_yaw;

    if (rx_complete_flag != 0U)
    {
        current_state = STATE_UART_PARSE;
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - last_yaw_tick) < STATE_HWT101_PERIOD_MS)
    {
        return;
    }
    last_yaw_tick = now;

    if (WT101_ReadYaw(&current_yaw,
                      WT101_I2C_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        motor_stop_all();
        return;
    }

    if (control_packet_received == 0U)
    {
        motor_stop_all();
        return;
    }

    mecanum_with_heading_control(motor_vx, motor_vy,
                                 target_yaw, current_yaw);
}

static void State_ParseUart(void)
{
    UART_Parse_Data();
    UART_Launch();
    control_packet_received = 1U;
    current_state = STATE_IDLE;
}

void State_Machine_Init(void)
{
    current_state = STATE_INIT;
    control_packet_received = 0U;
    last_yaw_tick = 0U;
}

void State_Machine_Update(void)
{
    switch (current_state)
    {
        case STATE_INIT:
            State_InitHardware();
            break;

        case STATE_IDLE:
            State_RunIdle();
            break;

        case STATE_UART_PARSE:
            State_ParseUart();
            break;

        case STATE_ERROR:
        default:
            State_EnterError();
            break;
    }
}

System_State_t State_Machine_GetCurrentState(void)
{
    return current_state;
}
