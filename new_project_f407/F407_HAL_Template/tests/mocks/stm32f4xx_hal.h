#ifndef MOCK_HAL
#define MOCK_HAL
#include <stdint.h>
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY} HAL_StatusTypeDef;
typedef struct {volatile uint32_t BSRR;} GPIO_TypeDef;
typedef struct {volatile uint32_t CR1,PSC,ARR,EGR,SR,DIER;} TIM_TypeDef;
typedef struct {uint32_t Pin,Mode,Pull,Speed;} GPIO_InitTypeDef;
extern GPIO_TypeDef ga,gd,ge;extern TIM_TypeDef t7;
#define GPIOA (&ga)
#define GPIOD (&gd)
#define GPIOE (&ge)
#define TIM7 (&t7)
#define GPIO_PIN_0 1U
#define GPIO_PIN_8 256U
#define GPIO_PIN_9 512U
#define GPIO_PIN_15 32768U
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_PULLDOWN 1
#define GPIO_SPEED_FREQ_HIGH 1
#define TIM_EGR_UG 1
#define TIM_SR_UIF 1
#define TIM_DIER_UIE 1
#define TIM_CR1_CEN 1
#define TIM7_IRQn 1
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM7_CLK_ENABLE() ((void)0)
#define __HAL_DBGMCU_FREEZE_TIM7() ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define HAL_RCC_GetPCLK1Freq() 42000000U
#define HAL_RCC_GetHCLKFreq() 168000000U
#define HAL_GPIO_Init(p,g) ((void)(p),(void)(g))
#define HAL_NVIC_SetPriority(a,b,c) ((void)0)
#define HAL_NVIC_ClearPendingIRQ(a) ((void)0)
#define HAL_NVIC_EnableIRQ(a) ((void)0)
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint32_t pin,unsigned high){p->BSRR=high?pin:pin<<16;}
extern uint32_t tick_ms;
#define HAL_GetTick() tick_ms
#endif
