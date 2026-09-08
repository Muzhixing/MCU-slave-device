/**
 ******************************************************************************
 * @file    test_upper_computer.c
 * @brief   Host regression test for the original upper-computer UART packet
 * @pin_resources PA2=USART2_TX and PA3=USART2_RX on target; none on host.
 * @peripherals Models the USART2 receive-complete callback.
 * @function Feeds exact bytes and verifies all ten packet fields and commands.
 * @purpose Prevents packet layout, values or receive flow from changing.
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "UART.h"
#include "UpperComputer.h"

GPIO_TypeDef test_gpioa = {1U};
GPIO_TypeDef test_gpiob = {2U};
uint32_t test_usart2_instance;
uint32_t test_usart1_instance;
uint32_t test_tim1_instance;
uint32_t test_tim2_instance;
UART_HandleTypeDef huart2 = {0};
static uint8_t *armed_byte;
static uint8_t gear_call_count;
static uint8_t rail_call_count;
static uint8_t gear_move_mm;
static uint8_t gear_addr;
static uint8_t gear_dir;
static uint16_t gear_vel;
static uint8_t gear_acc;
static uint8_t gear_raF;
static uint8_t gear_snF;
static uint8_t rail_move_mm;
static uint8_t rail_addr;
static uint8_t rail_dir;
static uint16_t rail_vel;
static uint8_t rail_acc;
static uint8_t rail_raF;
static uint8_t rail_snF;
static uint8_t delay_one_ms_count;
static uint32_t test_tick;

void UART_SendString(const char *text)
{
    (void)text;
}

void Gear_StepMotor_ControlByMM(uint8_t move_mm, uint8_t addr, uint8_t dir,
                                uint16_t vel, uint8_t acc, uint8_t raF,
                                uint8_t snF)
{
    gear_call_count++;
    gear_move_mm = move_mm;
    gear_addr = addr;
    gear_dir = dir;
    gear_vel = vel;
    gear_acc = acc;
    gear_raF = raF;
    gear_snF = snF;
}

void Rail_StepMotor_ControlByMM(uint8_t move_mm, uint8_t addr, uint8_t dir,
                                uint16_t vel, uint8_t acc, uint8_t raF,
                                uint8_t snF)
{
    rail_call_count++;
    rail_move_mm = move_mm;
    rail_addr = addr;
    rail_dir = dir;
    rail_vel = vel;
    rail_acc = acc;
    rail_raF = raF;
    rail_snF = snF;
}

void HAL_Delay(uint32_t delay_ms)
{
    if (delay_ms == 1U)
    {
        delay_one_ms_count++;
    }
}

uint32_t HAL_GetTick(void)
{
    return test_tick;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port;
    (void)init;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state)
{
    (void)port;
    (void)pins;
    (void)state;
}

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart)
{
    (void)huart;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                     uint8_t *data,
                                     uint16_t length)
{
    (void)huart;
    if (length != 1U)
    {
        return HAL_ERROR;
    }
    armed_byte = data;
    return HAL_OK;
}

static void feed(const uint8_t *bytes, uint32_t length)
{
    uint32_t index;
    for (index = 0U; index < length; index++)
    {
        if (armed_byte == NULL)
        {
            fputs("FAIL: USART2 receive not armed\n", stderr);
            exit(1);
        }
        *armed_byte = bytes[index];
        armed_byte = NULL;
        HAL_UART_RxCpltCallback(&huart2);
    }
}

int main(void)
{
    const uint8_t packet[UART_PACKET_LENGTH] =
        {0xB3U, 11U, 22U, 33U, 44U, 55U, 66U, 77U, 88U, 0xB4U};
    const UART_Packet_t *parsed;

    huart2.Instance = USART2;
    UART_Enable_Receive();
    feed(packet, UART_PACKET_LENGTH);
    if (rx_complete_flag == 0U)
    {
        fputs("FAIL: valid ten-byte packet not completed\n", stderr);
        return 1;
    }

    UART_Parse_Data();
    UART_Launch();
    parsed = UART_GetLatestPacket();

    if ((parsed->header != 0xB3U) ||
        (parsed->forward_speed != 11U) ||
        (parsed->horizontal_speed != 22U) ||
        (parsed->target_angle != 33U) ||
        (parsed->rudder_angle != 44U) ||
        (parsed->lift_rod != 55U) ||
        (parsed->horizontal_rod != 66U) ||
        (parsed->switch_one != 77U) ||
        (parsed->switch_two != 88U) ||
        (parsed->footer != 0xB4U))
    {
        fputs("FAIL: original UART packet fields changed\n", stderr);
        return 1;
    }

    if ((motor_vx != 11U) || (motor_vy != 22U) || (target_yaw != 33.0f))
    {
        fputs("FAIL: original chassis command assignment changed\n", stderr);
        return 1;
    }

    if ((gear_call_count != 1U) || (gear_move_mm != 66U) ||
        (gear_addr != 2U) || (gear_dir != 0U) || (gear_vel != 10U) ||
        (gear_acc != 5U) || (gear_raF != 1U) || (gear_snF != 0U))
    {
        fputs("FAIL: horizontal command no longer matches original CAN data\n",
              stderr);
        return 1;
    }
    if ((rail_call_count != 1U) || (rail_move_mm != 55U) ||
        (rail_addr != 1U) || (rail_dir != 1U) || (rail_vel != 100U) ||
        (rail_acc != 5U) || (rail_raF != 1U) || (rail_snF != 0U))
    {
        fputs("FAIL: lift command no longer matches original CAN data\n",
              stderr);
        return 1;
    }
    if (delay_one_ms_count != 1U)
    {
        fputs("FAIL: original 1 ms separation between axes was removed\n",
              stderr);
        return 1;
    }

    UART_Launch();
    if ((gear_call_count != 1U) || (rail_call_count != 1U))
    {
        fputs("FAIL: unchanged packet must not resend position commands\n",
              stderr);
        return 1;
    }

    {
        const uint8_t binary_with_header_like_payload[UART_PACKET_LENGTH] =
            {0xB3U, 0x5BU, 0xB3U, 3U, 4U, 0U, 0U, 7U, 8U, 0xB4U};

        feed(binary_with_header_like_payload, UART_PACKET_LENGTH);
        if (rx_complete_flag == 0U)
        {
            fputs("FAIL: binary payload bytes must not restart framing\n",
                  stderr);
            return 1;
        }
        UART_Parse_Data();
        parsed = UART_GetLatestPacket();
        if ((parsed->forward_speed != 0x5BU) ||
            (parsed->horizontal_speed != 0xB3U))
        {
            fputs("FAIL: binary payload bytes changed during framing\n",
                  stderr);
            return 1;
        }
    }

    {
        static const uint8_t centered[] = "[j,0,0,0,0]";
        static const uint8_t positive[] = "[j,0,100,0,100]";
        static const uint8_t negative[] = "[joystick,0,-60,0,-40]";

        feed(centered, sizeof(centered) - 1U);
        if (rx_complete_flag == 0U)
        {
            fputs("FAIL: centered joystick packet not completed\n", stderr);
            return 1;
        }
        UART_Parse_Data();
        UART_Launch();
        if ((gear_call_count != 1U) || (rail_call_count != 1U))
        {
            fputs("FAIL: centered joysticks must not move either axis\n", stderr);
            return 1;
        }

        feed(positive, sizeof(positive) - 1U);
        UART_Parse_Data();
        UART_Launch();
        if ((rail_call_count != 2U) || (rail_move_mm != 5U) ||
            (rail_addr != 1U) || (rail_dir != 1U) || (rail_vel != 30U) ||
            (rail_acc != 5U) || (rail_raF != 0U) || (rail_snF != 0U))
        {
            fputs("FAIL: left joystick up must move steel wire +5 mm\n",
                  stderr);
            return 1;
        }
        if ((gear_call_count != 2U) || (gear_move_mm != 5U) ||
            (gear_addr != 2U) || (gear_dir != 0U) || (gear_vel != 10U) ||
            (gear_acc != 5U) || (gear_raF != 0U) || (gear_snF != 0U))
        {
            fputs("FAIL: right joystick up must move crossbar +5 mm\n",
                  stderr);
            return 1;
        }

        feed(positive, sizeof(positive) - 1U);
        UART_Parse_Data();
        UART_Launch();
        if ((gear_call_count != 2U) || (rail_call_count != 2U))
        {
            fputs("FAIL: held joysticks repeated before motion interval\n",
                  stderr);
            return 1;
        }

        test_tick = 2600U;
        feed(positive, sizeof(positive) - 1U);
        UART_Parse_Data();
        UART_Launch();
        if ((gear_call_count != 3U) || (rail_call_count != 3U))
        {
            fputs("FAIL: held joysticks must repeat after motion interval\n",
                  stderr);
            return 1;
        }

        feed(centered, sizeof(centered) - 1U);
        UART_Parse_Data();
        UART_Launch();
        feed(negative, sizeof(negative) - 1U);
        UART_Parse_Data();
        UART_Launch();
        if ((rail_call_count != 4U) || (rail_move_mm != 2U) ||
            (rail_dir != 0U))
        {
            fputs("FAIL: left joystick down must move steel wire -2 mm\n",
                  stderr);
            return 1;
        }
        if ((gear_call_count != 4U) || (gear_move_mm != 1U) ||
            (gear_dir != 1U))
        {
            fputs("FAIL: right joystick down must move crossbar -1 mm\n",
                  stderr);
            return 1;
        }


        feed(centered, sizeof(centered) - 1U);
        UART_Parse_Data();
        UART_Launch();
        feed(positive, sizeof(positive) - 1U);
        UART_Parse_Data();
        UART_Launch();
        {
            uint8_t rail_count_before_long_move = rail_call_count;
            uint8_t gear_count_before_long_move = gear_call_count;

            while (test_tick < 93000U)
            {
                test_tick += 2600U;
                feed(positive, sizeof(positive) - 1U);
                UART_Parse_Data();
                UART_Launch();
            }
            if ((rail_call_count <= rail_count_before_long_move) ||
                (gear_call_count <= gear_count_before_long_move) ||
                (rail_move_mm != 5U) || (gear_move_mm != 5U))
            {
                fputs("FAIL: held joysticks must continue beyond 180 mm\n",
                      stderr);
                return 1;
            }
        }
        {
            uint8_t rail_count_before_next = rail_call_count;
            uint8_t gear_count_before_next = gear_call_count;

            test_tick += 2600U;
            feed(positive, sizeof(positive) - 1U);
            UART_Parse_Data();
            UART_Launch();
            if ((rail_call_count != (uint8_t)(rail_count_before_next + 1U)) ||
                (gear_call_count != (uint8_t)(gear_count_before_next + 1U)))
            {
                fputs("FAIL: no cumulative joystick travel limit expected\n",
                      stderr);
                return 1;
            }
        }
    }

    puts("PASS: original binary and Bluetooth joystick UART data flow");
    return 0;
}
