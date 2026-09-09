/**
 ******************************************************************************
 * @file    UpperComputer.c
 * @brief   Original upper-computer packet receive and assignment logic
 * @pin_resources PA2=USART2_TX and PA3=USART2_RX through UART.c.
 * @peripherals USART2 receive interrupt.
 * @function Routes B3/B4 binary packets and [j,LX,LY,RX,RY] joystick packets.
 * @purpose Preserves the original serial controls and adds safe two-axis tests.
 * @migration The original ten-byte layout and CAN motion parameters are kept.
 ******************************************************************************
 */

#include "UpperComputer.h"

#include "UART.h"
#include "tower.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define JOYSTICK_DEAD_ZONE       20L
#define STEEL_WIRE_ADDR          1U
#define STEEL_WIRE_POSITIVE_DIR  1U
#define STEEL_WIRE_NEGATIVE_DIR  0U
#define STEEL_WIRE_VELOCITY      30U
#define CROSSBAR_ADDR            2U
#define CROSSBAR_POSITIVE_DIR    0U
#define CROSSBAR_NEGATIVE_DIR    1U
#define CROSSBAR_VELOCITY        10U
#define STEPPER_ACCELERATION     5U
#define STEEL_WIRE_MS_PER_MM     500U
#define STEEL_WIRE_SETTLE_MS     100U
#define CROSSBAR_MS_PER_MM       48U
#define CROSSBAR_SETTLE_MS       50U
#define CROSSBAR_MIN_PERIOD_MS   200U

typedef enum
{
    UART_RECEIVE_IDLE = 0,
    UART_RECEIVE_BINARY,
    UART_RECEIVE_JOYSTICK
} UART_ReceiveMode_t;

typedef enum
{
    UART_COMMAND_NONE = 0,
    UART_COMMAND_BINARY,
    UART_COMMAND_JOYSTICK
} UART_CommandType_t;

typedef struct
{
    int32_t left_x;
    int32_t left_y;
    int32_t right_x;
    int32_t right_y;
} UART_Joystick_t;

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
static UART_Joystick_t joystick_packet;
static UART_ReceiveMode_t receive_mode;
static UART_CommandType_t command_type;
static uint32_t steel_wire_next_move_tick;
static uint32_t crossbar_next_move_tick;

static void UpperComputer_ReceiveByte(uint8_t byte);

static int32_t UART_Clamp_Joystick(long value)
{
    if (value > 100L)
    {
        return 100;
    }
    if (value < -100L)
    {
        return -100;
    }
    return (int32_t)value;
}

static uint8_t UART_Joystick_Distance(int32_t value)
{
    uint32_t magnitude = (uint32_t)((value < 0) ? -value : value);

    if (magnitude <= (uint32_t)JOYSTICK_DEAD_ZONE)
    {
        return 0U;
    }
    if (magnitude <= 40U)
    {
        return 1U;
    }
    if (magnitude <= 60U)
    {
        return 2U;
    }
    if (magnitude <= 80U)
    {
        return 3U;
    }
    return 5U;
}

static uint8_t UART_Time_Reached(uint32_t now, uint32_t deadline)
{
    return (uint8_t)((int32_t)(now - deadline) >= 0);
}

static uint32_t UART_Steel_Wire_Period(uint8_t distance)
{
    return ((uint32_t)distance * STEEL_WIRE_MS_PER_MM) +
           STEEL_WIRE_SETTLE_MS;
}

static uint32_t UART_Crossbar_Period(uint8_t distance)
{
    uint32_t period = ((uint32_t)distance * CROSSBAR_MS_PER_MM) +
                      CROSSBAR_SETTLE_MS;

    if (period < CROSSBAR_MIN_PERIOD_MS)
    {
        period = CROSSBAR_MIN_PERIOD_MS;
    }
    return period;
}

static uint8_t UART_Parse_Joystick(const uint8_t *buffer,
                                   uint8_t length,
                                   UART_Joystick_t *joystick)
{
    const char *cursor;
    char *end;
    long values[4];
    uint8_t index;

    if ((buffer == NULL) || (joystick == NULL) || (length < 11U))
    {
        return 0U;
    }

    if (strncmp((const char *)buffer, "[j,", 3U) == 0)
    {
        cursor = (const char *)buffer + 3;
    }
    else if (strncmp((const char *)buffer, "[joystick,", 10U) == 0)
    {
        cursor = (const char *)buffer + 10;
    }
    else
    {
        return 0U;
    }

    for (index = 0U; index < 4U; index++)
    {
        values[index] = strtol(cursor, &end, 10);
        if (end == cursor)
        {
            return 0U;
        }
        if (index < 3U)
        {
            if (*end != ',')
            {
                return 0U;
            }
            cursor = end + 1;
        }
        else if ((end[0] != ']') || (end[1] != '\0'))
        {
            return 0U;
        }
    }

    joystick->left_x = UART_Clamp_Joystick(values[0]);
    joystick->left_y = UART_Clamp_Joystick(values[1]);
    joystick->right_x = UART_Clamp_Joystick(values[2]);
    joystick->right_y = UART_Clamp_Joystick(values[3]);
    return 1U;
}

static void UART_Report_Move(const char *motor, int32_t joystick_value,
                             uint8_t direction, uint8_t distance)
{
    char text[72];

    (void)snprintf(text, sizeof(text), "%s JOY=%ld DIR=%u MM=%u\r\n",
                   motor, (long)joystick_value, direction, distance);
    UART_SendString(text);
}

static uint8_t UART_Validate_Packet(const uint8_t *buffer, uint8_t length)
{
    if (length != UART_PACKET_LENGTH)
    {
        return 0U;
    }
    if (buffer[0] != UART_PACKET_HEADER)
    {
        return 0U;
    }
    if (buffer[UART_PACKET_LENGTH - 1U] != UART_PACKET_FOOTER)
    {
        return 0U;
    }
    return 1U;
}

void UART_Enable_Receive(void)
{
    rx_complete_flag = 0U;
    rx_cnt = 0U;
    receive_mode = UART_RECEIVE_IDLE;
    command_type = UART_COMMAND_NONE;
    steel_wire_next_move_tick = HAL_GetTick();
    crossbar_next_move_tick = HAL_GetTick();
    UART_StartReceiveIT(UpperComputer_ReceiveByte);
}

void UART_Disable_Receive(void)
{
    parsing_in_progress = 1U;
}

static void UpperComputer_ReceiveByte(uint8_t byte)
{
    rx_data = byte;

    if (parsing_in_progress != 0U)
    {
        return;
    }

    if ((receive_mode == UART_RECEIVE_IDLE) &&
        ((rx_data == UART_PACKET_HEADER) ||
         (rx_data == (uint8_t)'[')))
    {
        rx_cnt = 0U;
        receive_mode = (rx_data == UART_PACKET_HEADER) ?
                       UART_RECEIVE_BINARY : UART_RECEIVE_JOYSTICK;
    }

    if (receive_mode == UART_RECEIVE_IDLE)
    {
        return;
    }

    if (rx_cnt >= (UART_RX_BUFFER_SIZE - 1U))
    {
        rx_cnt = 0U;
        receive_mode = UART_RECEIVE_IDLE;
        return;
    }

    rx_buffer[rx_cnt++] = rx_data;
    if ((receive_mode == UART_RECEIVE_BINARY) &&
        (rx_cnt == UART_PACKET_LENGTH))
    {
        if (UART_Validate_Packet(rx_buffer, rx_cnt) != 0U)
        {
            rx_complete_flag = 1U;
        }
        else
        {
            rx_cnt = 0U;
            receive_mode = UART_RECEIVE_IDLE;
        }
    }
    else if ((receive_mode == UART_RECEIVE_JOYSTICK) &&
             (rx_data == (uint8_t)']'))
    {
        rx_buffer[rx_cnt] = '\0';
        rx_complete_flag = 1U;
    }
}

void UART_Parse_Data(void)
{
    if (rx_complete_flag == 0U)
    {
        return;
    }

    parsing_in_progress = 1U;
    command_type = UART_COMMAND_NONE;
    if ((receive_mode == UART_RECEIVE_BINARY) &&
        (UART_Validate_Packet(rx_buffer, rx_cnt) != 0U))
    {
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
        command_type = UART_COMMAND_BINARY;
    }
    else if ((receive_mode == UART_RECEIVE_JOYSTICK) &&
             (UART_Parse_Joystick(rx_buffer, rx_cnt,
                                  &joystick_packet) != 0U))
    {
        command_type = UART_COMMAND_JOYSTICK;
    }
    parsing_in_progress = 0U;
    rx_cnt = 0U;
    receive_mode = UART_RECEIVE_IDLE;
    rx_complete_flag = 0U;
}

void UART_Launch(void)
{
    uint8_t distance;
    uint32_t now;

    if (command_type == UART_COMMAND_JOYSTICK)
    {
        now = HAL_GetTick();
        distance = UART_Joystick_Distance(joystick_packet.left_y);
        if (distance == 0U)
        {
            steel_wire_next_move_tick = now;
        }
        else if (UART_Time_Reached(now, steel_wire_next_move_tick) != 0U)
        {
            uint8_t direction = (joystick_packet.left_y > 0) ?
                                STEEL_WIRE_POSITIVE_DIR :
                                STEEL_WIRE_NEGATIVE_DIR;
            UART_Report_Move("STEEL_WIRE", joystick_packet.left_y,
                             direction, distance);
            Rail_StepMotor_ControlByMM(distance, STEEL_WIRE_ADDR, direction,
                                       STEEL_WIRE_VELOCITY,
                                       STEPPER_ACCELERATION, false, false);
            steel_wire_next_move_tick = now +
                                        UART_Steel_Wire_Period(distance);
        }

        HAL_Delay(1U);

        distance = UART_Joystick_Distance(joystick_packet.right_y);
        if (distance == 0U)
        {
            crossbar_next_move_tick = now;
        }
        else if (UART_Time_Reached(now, crossbar_next_move_tick) != 0U)
        {
            uint8_t direction = (joystick_packet.right_y > 0) ?
                                CROSSBAR_POSITIVE_DIR :
                                CROSSBAR_NEGATIVE_DIR;
            UART_Report_Move("CROSSBAR", joystick_packet.right_y,
                             direction, distance);
            Gear_StepMotor_ControlByMM(distance, CROSSBAR_ADDR, direction,
                                       CROSSBAR_VELOCITY,
                                       STEPPER_ACCELERATION, false, false);
            crossbar_next_move_tick = now + UART_Crossbar_Period(distance);
        }
        return;
    }

    if (command_type != UART_COMMAND_BINARY)
    {
        return;
    }

    motor_vx = rx_packet.forward_speed;
    motor_vy = rx_packet.horizontal_speed;
    target_yaw = rx_packet.target_angle;

    if (rx_packet.horizontal_rod != last_packet.horizontal_rod)
    {
        UART_Report_Move("BINARY_CROSSBAR", rx_packet.horizontal_rod,
                         CROSSBAR_POSITIVE_DIR,
                         rx_packet.horizontal_rod);
        Gear_StepMotor_ControlByMM(rx_packet.horizontal_rod, 2U, 0U,
                                   10U, 5U, true, false);
        last_packet.horizontal_rod = rx_packet.horizontal_rod;
    }

    HAL_Delay(1U);

    if (rx_packet.lift_rod != last_packet.lift_rod)
    {
        UART_Report_Move("BINARY_STEEL_WIRE", rx_packet.lift_rod,
                         STEEL_WIRE_POSITIVE_DIR, rx_packet.lift_rod);
        Rail_StepMotor_ControlByMM(rx_packet.lift_rod, 1U, 1U,
                                   100U, 5U, true, false);
        last_packet.lift_rod = rx_packet.lift_rod;
    }
}

const UART_Packet_t *UART_GetLatestPacket(void)
{
    return &rx_packet;
}
