/**
 ******************************************************************************
 * @file    main.c
 * @brief   STM32F103C8 Bluetooth dual-stepper CAN test
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
 *   - Homes the steel-wire motor at CAN address 1 after MCU power-up.
 *   - Receives Bluetooth joystick and original B3/B4 packets on USART2.
 *   - Left joystick Y controls steel wire; right joystick Y controls crossbar.
 *
 * @purpose
 *   - No Bluetooth motion command is accepted during the homing timeout.
 *   - Holding a joystick repeats relative moves after each motion interval.
 *   - Returning the joystick to centre stops issuing additional moves.
 ******************************************************************************
 */

#include "stm32f1xx_hal.h"
#include "UART.h"
#include "can.h"
#include "hcan.h"
#include "Emm_V5.h"
#include "UpperComputer.h"

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
    UART_SendString("BLUETOOTH DUAL STEPPER TEST BAUD=500000 ADDR1=STEEL ADDR2=CROSSBAR\r\n");
    CAN_PrintStatus("CAN START");

    can_rx_received = 0U;
    UART_SendString("WAIT DRIVER POWER-UP 1000MS\r\n");
    HAL_Delay(1000U);

    UART_SendString("SET HOMING MODE=2 DIR=0 VEL=30 TIMEOUT=20000 SL_RPM=30 SL_MA=800 SL_MS=60 POT=0\r\n");
    Emm_V5_Origin_Modify_Params(1U, false, 2U, 0U, 30U, 20000U,
                                30U, 800U, 60U, false);
    HAL_Delay(100U);
    CAN_PrintStatus("CAN HOME PARAM");

    UART_SendString("ENABLE LEAD_SCREW=1\r\n");
    Emm_V5_En_Control(1U, true, false);
    HAL_Delay(100U);
    CAN_PrintStatus("CAN ENABLE");

    UART_SendString("TRIGGER SENSORLESS HOMING MODE=2\r\n");
    Emm_V5_Origin_Trigger_Return(1U, 2U, false);
    HAL_Delay(200U);
    CAN_PrintStatus("CAN HOME TRIGGER");
    CAN_PrintReceivedFrame();

    UART_SendString("WAIT HOMING WINDOW 20000MS - CONTROL LOCKED\r\n");
    HAL_Delay(20000U);

    UART_SendString("ENABLE CROSSBAR=2\r\n");
    Emm_V5_En_Control(2U, true, false);
    HAL_Delay(100U);
    CAN_PrintStatus("CAN CROSSBAR ENABLE");

    UART_Enable_Receive();
    UART_SendString("CONTROL READY: [j,LX,LY,RX,RY] OR B3...B4\r\n");

    while (1)
    {
        if (rx_complete_flag != 0U)
        {
            UART_Disable_Receive();
            UART_Parse_Data();
            UART_Launch();
        }
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
