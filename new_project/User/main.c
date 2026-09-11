/**
 ******************************************************************************
 * @file    main.c
 * @brief   Formal calf-island feeding vehicle controller entry
 *
 * @pin_resources
 *   - HC-08 USART2: PA2 TX, PA3 RX.
 *   - HWT101 I2C1: PB6 SCL, PB7 SDA.
 *   - CAN1: PA11 RX, PA12 TX through an external transceiver.
 *   - Horizontal servo: PA7/TIM3_CH2.
 *   - Four chassis motor pins are configured by Motor.c.
 *
 * @function
 *   - Initializes HAL and the 72 MHz clock, then runs the single formal state
 *     machine. Experimental mode selection is intentionally absent.
 ******************************************************************************
 */

#include "stm32f1xx_hal.h"
#include "State_Machine.h"

static void SystemClock_Config(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    State_Machine_Init();

    while (1)
    {
        State_Machine_Update();
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK |
                       RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 |
                       RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV2;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2) != HAL_OK)
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
