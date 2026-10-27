#include "water_pump.h"
static uint8_t ready;
void Pump_Off(void) {
 if(ready) HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
}
void Pump_Init(void) {
 GPIO_InitTypeDef gpio={0};
 __HAL_RCC_GPIOE_CLK_ENABLE();
 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
 gpio.Pin=GPIO_PIN_1;gpio.Mode=GPIO_MODE_OUTPUT_PP;
 gpio.Pull=GPIO_PULLDOWN;gpio.Speed=GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOE,&gpio);ready=1;Pump_Off();
}
