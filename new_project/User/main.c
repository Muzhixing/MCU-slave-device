/**
 ******************************************************************************
 * @file    main.c
 * @brief   Open-loop four-wheel chassis with selectable CAN stepper test
 *
 * @pin_resources
 *   - UART: PA2 TX, PA3 RX, PA6 RS485 DE, PA7 RS485 /RE.
 *   - Motor PWM: PA0, PA1, PA8 and PA11.
 *   - Motor direction: PA4, PB3, PB4, PB5, PB12, PB13, PB14 and PB15.
 *   - HWT101 UART: PB6 USART1_TX and PB7 USART1_RX after USART1 remap.
 *   - Bluetooth activity LED: PC13 output to an external LED through 510 ohm.
 *   - CAN mode: PA11=CAN1_RX and PA12=CAN1_TX through an external transceiver.
 *   - PA13 : SWDIO, reserved for programming and debugging.
 *   - PA14 : SWCLK, reserved for programming and debugging.
 *
 * @peripherals
 *   - RCC, SysTick, USART1, USART2, TIM1, TIM2, GPIOA, GPIOB and AFIO.
 *
 * @function
 *   - UART mode: sends the verified ready text, then echoes received bytes.
 *   - Motor mode: PWM 500 forward 10 s, reverse 10 s, continuously.
 *   - WT101 mode: forwards 11-byte raw frames and checksum state through USART2.
 *   - Bluetooth mode: maps joystick packets directly to Mecanum chassis motion.
 *
 * @purpose
 *   - Independently verifies each migrated interface and chassis function.
 *
 * @migration
 *   - Sources: verified E:\project_M\test_p UART and motor tests.
 *   - HWT101 input and USART2 debug output: 115200 8N1 on separate UARTs.
 *   - Unchanged: PWM 500 and 10-second direction intervals.
 *   - Default is the four-wheel open-loop Bluetooth chassis from d0c4b19.
 *   - CAN stepper mode is selectable and remains isolated from PA11 motor PWM.
 ******************************************************************************
 */

#include "stm32f1xx_hal.h"
#include "App_Test.h"
#include "buleteethtest.h"
#include "UART.h"
#include "can.h"
#include "hcan.h"
#include "Emm_V5.h"
#include "UpperComputer.h"

#define MAIN_MODE_OPEN_LOOP_CHASSIS  1U
#define MAIN_MODE_CAN_STEPPER        2U

#ifndef MAIN_APP_MODE
#define MAIN_APP_MODE MAIN_MODE_OPEN_LOOP_CHASSIS
#endif

static void SystemClock_Config(void);
void Error_Handler(void);
static void UART_SendHex8(uint8_t value);
static void UART_SendHex32(uint32_t value);
static void CAN_PrintStatus(const char *label);
static void CAN_PrintReceivedFrame(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

#if MAIN_APP_MODE == MAIN_MODE_OPEN_LOOP_CHASSIS
#if APP_TEST_MODE == APP_TEST_UART
    App_Test_UART_Init();

    while (1)
    {
        App_Test_UART_ProcessByte();
    }
#elif APP_TEST_MODE == APP_TEST_MOTOR
    App_Test_Motor_Init();

    while (1)
    {
        App_Test_Motor_RunCycle();
    }
#elif APP_TEST_MODE == APP_TEST_WT101
    App_Test_WT101_Init();
    while (1)
    {
        App_Test_WT101_RunStep();
    }
#elif APP_TEST_MODE == APP_TEST_CONTROL
    App_Test_Control_Init();
    while (1)
    {
        App_Test_Control_RunStep();
    }
#elif APP_TEST_MODE == APP_TEST_BLUETOOTH
    BluetoothTest_Init();
    while (1)
    {
        BluetoothTest_RunStep();
    }
#else
#error "APP_TEST_MODE selection is invalid"
#endif
#elif MAIN_APP_MODE == MAIN_MODE_CAN_STEPPER
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
#else
#error "MAIN_APP_MODE selection is invalid"
#endif
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
