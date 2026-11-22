#ifndef HEADING_DRIVER_MOCK
#define HEADING_DRIVER_MOCK
#include <stdint.h>
typedef enum {HAL_OK,HAL_ERROR} HAL_StatusTypeDef;
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
typedef struct {volatile uint32_t SR,DR;} USART_TypeDef;
typedef struct {USART_TypeDef *Instance;struct {uint32_t BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,OverSampling;} Init;} UART_HandleTypeDef;
extern USART_TypeDef mock_uart;extern uint32_t tick_ms;
#define USART2 (&mock_uart)
#define GPIOA ((void*)0)
#define GPIO_PIN_3 8
#define GPIO_MODE_AF_PP 1
#define GPIO_PULLUP 1
#define GPIO_SPEED_FREQ_HIGH 1
#define GPIO_AF7_USART2 7
#define UART_WORDLENGTH_8B 8
#define UART_STOPBITS_1 1
#define UART_PARITY_NONE 0
#define UART_MODE_RX 1
#define UART_HWCONTROL_NONE 0
#define UART_OVERSAMPLING_16 16
#define USART2_IRQn 2
#define UART_IT_RXNE 1
#define UART_IT_ERR 2
#define USART_SR_PE 1
#define USART_SR_FE 2
#define USART_SR_NE 4
#define USART_SR_ORE 8
#define USART_SR_RXNE 32
#define HAL_NVIC_DisableIRQ(x) ((void)(x))
#define HAL_NVIC_ClearPendingIRQ(x) ((void)(x))
#define HAL_NVIC_EnableIRQ(x) ((void)(x))
#define HAL_NVIC_SetPriority(x,y,z) ((void)(x),(void)(y),(void)(z))
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART2_CLK_ENABLE() ((void)0)
#define HAL_GPIO_Init(x,y) ((void)(x),(void)(y))
#define __HAL_UART_CLEAR_OREFLAG(x) ((void)(x))
#define __HAL_UART_ENABLE_IT(x,y) ((void)(x),(void)(y))
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define HAL_GetTick() tick_ms
static inline HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *u){return u->Init.BaudRate==115200?HAL_OK:HAL_ERROR;}
#endif
