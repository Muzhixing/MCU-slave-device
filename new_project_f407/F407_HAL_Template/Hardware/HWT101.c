#include "HWT101.h"
#define RX_CAPACITY 256U
#define RX_MASK 255U
typedef struct { uint32_t ms; uint8_t byte; } RxByte;
static volatile RxByte rx[RX_CAPACITY];
static volatile uint16_t head, tail;
static volatile uint8_t lost;
static volatile uint32_t bytes, errors;
static UART_HandleTypeDef uart;
static HWT101_Parser parser;
HAL_StatusTypeDef HWT101_Init(void)
{
  GPIO_InitTypeDef gpio={0};
  HAL_NVIC_DisableIRQ(USART2_IRQn);
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  gpio.Pin=GPIO_PIN_3; gpio.Mode=GPIO_MODE_AF_PP;
  gpio.Pull=GPIO_PULLUP; gpio.Speed=GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate=GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA,&gpio);
  uart.Instance=USART2;
  uart.Init.BaudRate=115200;
  uart.Init.WordLength=UART_WORDLENGTH_8B;
  uart.Init.StopBits=UART_STOPBITS_1;
  uart.Init.Parity=UART_PARITY_NONE;
  uart.Init.Mode=UART_MODE_RX;
  uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  uart.Init.OverSampling=UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&uart)!=HAL_OK) return HAL_ERROR;
  head=tail=lost=0; bytes=errors=0;
  HWT101_ParserInit(&parser);
  __HAL_UART_CLEAR_OREFLAG(&uart);
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  HAL_NVIC_SetPriority(USART2_IRQn,3,0);
  __HAL_UART_ENABLE_IT(&uart,UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&uart,UART_IT_ERR);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  return HAL_OK;
}
/* F407 SR then DR read clears RXNE/ORE/FE/NE; no HAL receive callbacks. */
void USART2_IRQHandler(void)
{
  uint32_t status=USART2->SR;
  uint8_t byte;
  uint16_t next;
  if(!(status & (USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE))) return;
  byte=(uint8_t)USART2->DR; bytes++;
  if(status & (USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) { lost=1; errors++; return; }
  if(lost) return;
  next=(uint16_t)((head+1U)&RX_MASK);
  if(next==tail) { lost=1; errors++; return; }
  rx[head].byte=byte; rx[head].ms=HAL_GetTick(); head=next;
}
void HWT101_Poll(void)
{
  uint32_t budget=RX_CAPACITY;
  while(budget--) {
    uint32_t mask=__get_PRIMASK(), ms=0;
    uint8_t byte=0, available, broken;
    __disable_irq();
    broken=lost;
    if(broken) { tail=head; lost=0; }
    available=(uint8_t)(tail!=head);
    if(available) { byte=rx[tail].byte; ms=rx[tail].ms; tail=(uint16_t)((tail+1U)&RX_MASK); }
    __set_PRIMASK(mask);
    if(broken) HWT101_ParserBreak(&parser);
    if(!available) break;
    HWT101_ParserFeed(&parser,byte,ms);
  }
  HWT101_ParserExpire(&parser,HAL_GetTick());
}
uint8_t HWT101_Read(HWT101_Data *data)
{
  if(!data) return 0;
  HWT101_ParserExpire(&parser,HAL_GetTick());
  *data=parser.data; return data->valid;
}
uint8_t HWT101_Zero(void)
{
  HWT101_Poll(); return HWT101_ParserZero(&parser,HAL_GetTick());
}
uint32_t HWT101_RxBytes(void) { return bytes; }
uint32_t HWT101_RxErrors(void) { return errors; }
