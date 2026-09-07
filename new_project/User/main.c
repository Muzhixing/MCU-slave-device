/**
 ******************************************************************************
 * @file    main.c
 * @brief   STM32F103C8 lead-screw safe movement test
 *
 * @pin_resources
 *   - PA11: CAN1_RX from an external CAN transceiver RXD.
 *   - PA12: CAN1_TX to an external CAN transceiver TXD.
 *   - PA2 : USART2_TX to Bluetooth/USB-TTL RXD.
 *   - PA3 : USART2_RX from Bluetooth/USB-TTL TXD.
 *   - PA6 : onboard RS485-2 DE, held low by UART_Init.
 *   - PA7 : onboard RS485-2 /RE, held high by UART_Init.
 *   - PA13: SWDIO, reserved for programming/debugging.
 *   - PA14: SWCLK, reserved for programming/debugging.
 *
 * @function
 *   - Starts USART2 at 115200 baud and CAN1 at the original 500 kbit/s.
 *   - Enables only the lead screw at the original CAN address 1.
 *   - Lead screw: forward 5 mm, dir=1, vel=30 RPM, acc=5.
 *
 * @purpose
 *   - Safely verifies the reset lead-screw driver before restoring the
 *     original 50 mm / 100 RPM command.
 ******************************************************************************
 */

#include "stm32f1xx_hal.h"
#include "UART.h"
#include "can.h"
#include "hcan.h"
#include "Emm_V5.h"
#include "tower.h"

static void SystemClock_Config(void);
static void UART_SendHex8(uint8_t value);
static void UART_SendHex32(uint32_t value);
static void CAN_PrintStatus(const char *label);
static void CAN_PrintReceivedFrame(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    UART_Init(115200U);
    MX_CAN_Init();
    CAN_Start(CAN_NUM);
    UART_SendString("LEAD SCREW SAFE TEST READY BAUD=500000 ADDR=1\r\n");
    CAN_PrintStatus("CAN START");

    can_rx_received = 0U;
    UART_SendString("ENABLE LEAD_SCREW=1\r\n");
    Emm_V5_En_Control(1U, true, false);
    HAL_Delay(100U);
    CAN_PrintStatus("CAN ENABLE");

    UART_SendString("MOVE LEAD_SCREW1=FORWARD5MM/DIR1/VEL30/ACC5\r\n");
    Rail_StepMotor_ControlByMM(5U, 1U, 1U, 30U, 5U, true, false);
    HAL_Delay(200U);
    CAN_PrintStatus("CAN MOVE");
    CAN_PrintReceivedFrame();

    while (1)
    {
    }
}

static void UART_SendHex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    UART_SendByte((uint8_t)hex[(value >> 4) & 0x0FU]);
    UART_SendByte((uint8_t)hex[value & 0x0FU]);
}

static void UART_SendHex32(uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    int8_t shift;

    for (shift = 28; shift >= 0; shift -= 4)
    {
        UART_SendByte((uint8_t)hex[(value >> shift) & 0x0FU]);
    }
}

static void CAN_PrintStatus(const char *label)
{
    UART_SendString(label);
    UART_SendString(" FILTER=");
    UART_SendHex8((uint8_t)can_filter_status);
    UART_SendString(" NOTIFY=");
    UART_SendHex8((uint8_t)can_notify_status);
    UART_SendString(" START=");
    UART_SendHex8((uint8_t)can_start_status);
    UART_SendString(" TX=");
    UART_SendHex8((uint8_t)can_tx_status);
    UART_SendString(" STATE=");
    UART_SendHex8((uint8_t)HAL_CAN_GetState(&hcan));
    UART_SendString(" ERROR=0x");
    UART_SendHex32(HAL_CAN_GetError(&hcan));
    UART_SendString(" ESR=0x");
    UART_SendHex32(hcan.Instance->ESR);
    UART_SendString(" TSR=0x");
    UART_SendHex32(hcan.Instance->TSR);
    UART_SendString("\r\n");
}

static void CAN_PrintReceivedFrame(void)
{
    uint8_t index;

    if (can_rx_received == 0U)
    {
        UART_SendString("RX FRAME: NONE\r\n");
        return;
    }

    UART_SendString("RX FRAME ID=0x");
    UART_SendHex32(can_rx_header.ExtId);
    UART_SendString(" DLC=");
    UART_SendHex8((uint8_t)can_rx_header.DLC);
    UART_SendString(" DATA=");
    for (index = 0U; index < can_rx_header.DLC; index++)
    {
        UART_SendHex8(can_rx_data[index]);
        UART_SendByte((uint8_t)' ');
    }
    UART_SendString("\r\n");
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
