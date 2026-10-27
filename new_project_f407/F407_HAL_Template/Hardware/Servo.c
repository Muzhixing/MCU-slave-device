#include "Servo.h"
static TIM_HandleTypeDef timer;
static volatile uint16_t current=SERVO_RESET_TENTHS,target=SERVO_RESET_TENTHS;
static volatile uint8_t enabled;
HAL_StatusTypeDef Servo_Init(void) {
 GPIO_InitTypeDef gpio={0};
 if(HAL_RCC_GetPCLK1Freq()!=42000000U || HAL_RCC_GetHCLKFreq()!=168000000U)return HAL_ERROR;
 __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_TIM5_CLK_ENABLE();
 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
 gpio.Pin=GPIO_PIN_5;gpio.Mode=GPIO_MODE_OUTPUT_PP;gpio.Pull=GPIO_NOPULL;gpio.Speed=GPIO_SPEED_FREQ_HIGH;
 HAL_GPIO_Init(GPIOE,&gpio);
 timer.Instance=TIM5;timer.Init.Prescaler=83;timer.Init.CounterMode=TIM_COUNTERMODE_UP;
 timer.Init.Period=19999;timer.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
 timer.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
 if(HAL_TIM_Base_Init(&timer)!=HAL_OK)return HAL_ERROR;
 current=target=SERVO_RESET_TENTHS;enabled=1;
 TIM5->CCR1=Servo_AngleToPulse(SERVO_RESET_TENTHS);TIM5->SR=0;
 HAL_NVIC_SetPriority(TIM5_IRQn,2,0);HAL_NVIC_ClearPendingIRQ(TIM5_IRQn);HAL_NVIC_EnableIRQ(TIM5_IRQn);
 TIM5->DIER=TIM_DIER_UIE|TIM_DIER_CC1IE;
 __HAL_TIM_ENABLE(&timer);
 return HAL_OK;
}
void TIM5_IRQHandler(void) {
 uint32_t status=TIM5->SR;
 if(status&TIM_SR_CC1IF) {TIM5->SR=~TIM_SR_CC1IF;GPIOE->BSRR=(uint32_t)GPIO_PIN_5<<16;}
 if(status&TIM_SR_UIF) {
   uint16_t goal=target;
   TIM5->SR=~TIM_SR_UIF;
   GPIOE->BSRR=(uint32_t)GPIO_PIN_5<<16;
   if(!enabled)return;
   /* Skip a badly delayed frame, never emit a pulse whose falling edge passed. */
   if(TIM5->CNT>100U)return;
   if(current<goal)current=(uint16_t)(goal-current>20U?current+20U:goal);
   else if(current>goal)current=(uint16_t)(current-goal>20U?current-20U:goal);
   TIM5->CCR1=Servo_AngleToPulse(current);
   GPIOE->BSRR=GPIO_PIN_5;
 }
}
void Servo_Hold(void) {
 uint32_t mask=__get_PRIMASK();__disable_irq();target=current;__set_PRIMASK(mask);
}
void Servo_Disable(void) {
 uint32_t mask=__get_PRIMASK();__disable_irq();enabled=0;
 GPIOE->BSRR=(uint32_t)GPIO_PIN_5<<16;__set_PRIMASK(mask);
}

void Servo_SetTenths(uint16_t angle){if(angle<=1800U)target=angle;}
uint16_t Servo_CurrentTenths(void){return current;}

uint16_t Servo_AngleToPulse(uint16_t angle) {
 return (uint16_t)(500U+((uint32_t)angle*2000U+900U)/1800U);
}
