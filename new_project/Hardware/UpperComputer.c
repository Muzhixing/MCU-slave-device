/**
 ******************************************************************************
 * @file    UpperComputer.c
 * @brief   Formal original ten-byte upper-computer protocol
 *
 * @pin_resources
 *   - USART2: PA2 TX and PA3 RX through UART.c.
 *   - Horizontal servo: PA7/TIM3_CH2 through Servo.c.
 *   - Steel wire/crossbar: CAN1 PA11/PA12 through tower.c.
 *
 * @function
 *   - Receives only B3...B4 fixed ten-byte binary frames.
 *   - Preserves the original command field order and actuator parameters.
 ******************************************************************************
 */

#include "UpperComputer.h"

#include "Servo.h"
#include "UART.h"
#include "tower.h"

#include <string.h>

uint8_t rx_data;
uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t rx_cnt = 0U;
volatile uint8_t rx_complete_flag = 0U;
volatile uint8_t parsing_in_progress = 0U;
uint16_t motor_vx = 0U;
uint16_t motor_vy = 0U;
float target_yaw = 0.0f;

static UART_Packet_t rx_packet;
static UART_Packet_t last_packet;
static uint8_t command_pending;

static uint8_t UART_ValidatePacket(const uint8_t *packet, uint8_t length)
{
    return (uint8_t)((length == UART_PACKET_LENGTH) &&
                     (packet[0] == UART_PACKET_HEADER) &&
                     (packet[UART_PACKET_LENGTH - 1U] == UART_PACKET_FOOTER));
}

static void UpperComputer_ReceiveByte(uint8_t byte)
{
    if ((parsing_in_progress != 0U) || (rx_complete_flag != 0U))
    {
        return;
    }

    if (rx_cnt == 0U)
    {
        if (byte != UART_PACKET_HEADER)
        {
            return;
        }
        rx_buffer[rx_cnt++] = byte;
        return;
    }

    if (rx_cnt >= UART_PACKET_LENGTH)
    {
        rx_cnt = 0U;
        return;
    }

    rx_buffer[rx_cnt++] = byte;
    if (rx_cnt == UART_PACKET_LENGTH)
    {
        if (UART_ValidatePacket(rx_buffer, rx_cnt) != 0U)
        {
            rx_complete_flag = 1U;
        }
        else
        {
            rx_cnt = 0U;
        }
    }
}

void UART_Enable_Receive(void)
{
    rx_cnt = 0U;
    rx_complete_flag = 0U;
    parsing_in_progress = 0U;
    command_pending = 0U;
    (void)memset(&rx_packet, 0, sizeof(rx_packet));
    (void)memset(&last_packet, 0, sizeof(last_packet));
    UART_StartReceiveIT(UpperComputer_ReceiveByte);
}

void UART_Disable_Receive(void)
{
    parsing_in_progress = 1U;
}

void UART_Parse_Data(void)
{
    if ((rx_complete_flag == 0U) ||
        (UART_ValidatePacket(rx_buffer, rx_cnt) == 0U))
    {
        parsing_in_progress = 0U;
        return;
    }

    parsing_in_progress = 1U;
    rx_packet.header = rx_buffer[0];
    rx_packet.forward_speed = rx_buffer[1];
    rx_packet.horizontal_speed = rx_buffer[2];
    rx_packet.target_angle = rx_buffer[3];
    rx_packet.rudder_angle = rx_buffer[4];
    rx_packet.lift_rod = rx_buffer[5];
    rx_packet.horizontal_rod = rx_buffer[6];
    rx_packet.switch_one = rx_buffer[7];
    rx_packet.switch_two = rx_buffer[8];
    rx_packet.footer = rx_buffer[9];
    command_pending = 1U;

    rx_cnt = 0U;
    rx_complete_flag = 0U;
    parsing_in_progress = 0U;
}

void UART_Launch(void)
{
    if (command_pending == 0U)
    {
        return;
    }

    motor_vx = rx_packet.forward_speed;
    motor_vy = rx_packet.horizontal_speed;
    target_yaw = rx_packet.target_angle;
    Servo_SetAngle_2(rx_packet.rudder_angle);

    if (rx_packet.horizontal_rod != last_packet.horizontal_rod)
    {
        Gear_StepMotor_ControlByMM(rx_packet.horizontal_rod, 2U, 0U,
                                   10U, 5U, true, false);
        last_packet.horizontal_rod = rx_packet.horizontal_rod;
    }

    HAL_Delay(1U);

    if (rx_packet.lift_rod != last_packet.lift_rod)
    {
        Rail_StepMotor_ControlByMM(rx_packet.lift_rod, 1U, 1U,
                                   100U, 5U, true, false);
        last_packet.lift_rod = rx_packet.lift_rod;
    }

    command_pending = 0U;
}

const UART_Packet_t *UART_GetLatestPacket(void)
{
    return &rx_packet;
}
