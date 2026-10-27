#include "StepperPulse.h"
/* TIM7: 125us scheduler. GPIO pulses; TIM1/2 stay owned by chassis.
 * CN1 pin4 PE9 STEP, pin3 PE8 DIR; CN2 pin4 PA15 STEP, pin3 PD0 DIR.
 * DIR low: lift down, slide forward (user-confirmed terminology).
 */
typedef struct { volatile uint32_t left; uint32_t total; uint16_t half; volatile uint16_t wait;
  volatile uint8_t high; } PulseState;
static PulseState state[2];
static volatile uint8_t ready;
static volatile int32_t position[2];
static int32_t direction[2];
/* Slide half-period ramps from 80 to 10 scheduler ticks: 50 to 400Hz; lift 40 to 1: 100 to 4000Hz. */
static void Level(unsigned a,unsigned high) {
 GPIO_TypeDef *p=a?GPIOA:GPIOE;
 uint32_t pin=a?GPIO_PIN_15:GPIO_PIN_9;
 p->BSRR=high?pin:(pin<<16);
}
HAL_StatusTypeDef StepperPulse_Init(void) {
 GPIO_InitTypeDef g={0};
 if(HAL_RCC_GetPCLK1Freq()!=42000000U || HAL_RCC_GetHCLKFreq()!=168000000U)return HAL_ERROR;
 __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();__HAL_RCC_GPIOE_CLK_ENABLE();
 __HAL_RCC_TIM7_CLK_ENABLE();
 GPIOA->BSRR=(uint32_t)GPIO_PIN_15<<16;
 GPIOD->BSRR=(uint32_t)GPIO_PIN_0<<16;
 GPIOE->BSRR=(uint32_t)(GPIO_PIN_8|GPIO_PIN_9)<<16;
 g.Mode=GPIO_MODE_OUTPUT_PP;g.Pull=GPIO_PULLDOWN;g.Speed=GPIO_SPEED_FREQ_HIGH;
 g.Pin=GPIO_PIN_8|GPIO_PIN_9;HAL_GPIO_Init(GPIOE,&g);
 g.Pin=GPIO_PIN_15;HAL_GPIO_Init(GPIOA,&g);
 g.Pin=GPIO_PIN_0;HAL_GPIO_Init(GPIOD,&g);
 TIM7->CR1=0;TIM7->PSC=83;TIM7->ARR=124;TIM7->EGR=TIM_EGR_UG;TIM7->SR=0;
 ready=1;StepperPulse_StopAll();
 __HAL_DBGMCU_FREEZE_TIM7();
 HAL_NVIC_SetPriority(TIM7_IRQn,2,0);HAL_NVIC_ClearPendingIRQ(TIM7_IRQn);HAL_NVIC_EnableIRQ(TIM7_IRQn);
 TIM7->DIER=TIM_DIER_UIE;TIM7->CR1=TIM_CR1_CEN;
 return HAL_OK;
}
uint8_t StepperPulse_Busy(void) {
 return (state[0].left || state[0].high || state[0].wait ||
         state[1].left || state[1].high || state[1].wait)?1:0;
}
HAL_StatusTypeDef StepperPulse_Start(unsigned a,unsigned reverse,uint32_t pulses) {
 uint32_t mask;
 if(!ready || a>1 || !pulses || pulses>(a?4450U:140000U))return HAL_ERROR;
 mask=__get_PRIMASK();__disable_irq();
 if(StepperPulse_Busy()){__set_PRIMASK(mask);return HAL_BUSY;}
 HAL_GPIO_WritePin(a?GPIOD:GPIOE,a?GPIO_PIN_0:GPIO_PIN_8,reverse?GPIO_PIN_SET:GPIO_PIN_RESET);
 direction[a]=reverse?-1:1;
 state[a].high=0;state[a].left=pulses;state[a].total=pulses;state[a].half=40;
 state[a].wait=5; /* >=500us direction setup before the first edge */
 __set_PRIMASK(mask);return HAL_OK;
}
void StepperPulse_StopAll(void) {
 uint32_t mask=__get_PRIMASK();unsigned a;
 __disable_irq();
 for(a=0;a<2;a++){state[a].left=0;state[a].high=0;state[a].wait=0;if(ready)Level(a,0);}
 __set_PRIMASK(mask);
}
void TIM7_IRQHandler(void) {
 unsigned a;
 if(!(TIM7->SR&TIM_SR_UIF))return;
 TIM7->SR=~TIM_SR_UIF;
 for(a=0;a<2;a++) {
  PulseState *s=&state[a];
  if(s->wait && --s->wait)continue;
  if(s->high){Level(a,0);s->high=0;s->wait=s->half;}
  else if(s->left){
   uint32_t progress=s->total-s->left,remain=s->left-1U,stage;
   if(progress>remain)progress=remain;
   stage=progress/2U;if(stage>35U)stage=35U;
   if(a)s->half=(uint16_t)(80U-2U*stage);
   else {stage=progress/4U;if(stage>39U)stage=39U;s->half=(uint16_t)(40U-stage);}
   Level(a,1);s->high=1;s->left--;position[a]+=direction[a];s->wait=s->half;}
 }
}

int32_t StepperPulse_Position(unsigned axis){return axis<2?position[axis]:0;}
uint8_t StepperPulse_Zero(void){
 uint32_t mask=__get_PRIMASK();__disable_irq();
 if(StepperPulse_Busy()){__set_PRIMASK(mask);return 0;}
 position[0]=position[1]=0;__set_PRIMASK(mask);return 1;
}

uint8_t StepperPulse_AxisBusy(unsigned a){
 return a<2 && (state[a].left || state[a].high || state[a].wait);
}
uint8_t StepperPulse_ZeroAxis(unsigned a){
 uint32_t mask=__get_PRIMASK();__disable_irq();
 if(a>1 || StepperPulse_Busy()){__set_PRIMASK(mask);return 0;}
 position[a]=0;__set_PRIMASK(mask);return 1;
}
