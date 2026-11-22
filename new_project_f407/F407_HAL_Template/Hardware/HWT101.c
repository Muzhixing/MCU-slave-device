#include "HWT101.h"
#include <string.h>
#define RX_CAPACITY 256U
#define RX_MASK 255U
typedef struct { uint32_t ms; uint8_t byte; } RxByte;
static volatile RxByte rx[RX_CAPACITY];
static volatile uint16_t head, tail;
static volatile uint8_t lost;
static volatile uint32_t bytes, errors;
static UART_HandleTypeDef uart;
static HWT101_Parser parser;
volatile HWT101_Diagnostics g_hwt101_diagnostics;
static void RefreshDiagnostics(void) {
 g_hwt101_diagnostics.valid_frames=parser.valid_frames;
 g_hwt101_diagnostics.angle_frames=parser.data.sequence;
 g_hwt101_diagnostics.checksum_errors=parser.checksum_errors;
 g_hwt101_diagnostics.frame_gap_errors=parser.frame_gap_errors;
 g_hwt101_diagnostics.discarded_bytes=parser.discarded_bytes;
 g_hwt101_diagnostics.discontinuities=parser.data.discontinuity_sequence;
 g_hwt101_diagnostics.recoveries=parser.recoveries;
 g_hwt101_diagnostics.last_angle_ms=parser.data.timestamp_ms;
 g_hwt101_diagnostics.angle_age_ms=parser.received?HAL_GetTick()-parser.data.timestamp_ms:UINT32_MAX;
 g_hwt101_diagnostics.flags=(parser.data.valid?1U:0U)|(parser.data.continuous_valid?2U:0U)|
                           (parser.data.rate_valid?4U:0U)|(parser.broken?8U:0U);
 g_hwt101_diagnostics.last_break_reason=parser.last_break_reason;
}
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
  memset((void *)&g_hwt101_diagnostics,0,sizeof(g_hwt101_diagnostics));
  g_hwt101_diagnostics.version=1;
  HWT101_ParserInit(&parser);
  RefreshDiagnostics();
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
  g_hwt101_diagnostics.rx_bytes=bytes;
  g_hwt101_diagnostics.last_rx_ms=HAL_GetTick();
  if(status & (USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
    lost|=1U; errors++;g_hwt101_diagnostics.uart_errors++;
    if(status&USART_SR_ORE)g_hwt101_diagnostics.overrun_errors++;
    if(status&USART_SR_FE)g_hwt101_diagnostics.framing_errors++;
    if(status&USART_SR_NE)g_hwt101_diagnostics.noise_errors++;
    if(status&USART_SR_PE)g_hwt101_diagnostics.parity_errors++;
    g_hwt101_diagnostics.last_error_ms=HAL_GetTick();return;
  }
  if(lost) return;
  next=(uint16_t)((head+1U)&RX_MASK);
  if(next==tail) { lost|=2U; errors++;g_hwt101_diagnostics.queue_overflows++;
    g_hwt101_diagnostics.last_error_ms=HAL_GetTick();return; }
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
    if(broken) HWT101_ParserBreakReason(&parser,(broken&1U)?HWT101_BREAK_DRIVER:HWT101_BREAK_OVERFLOW);
    if(!available) break;
    HWT101_ParserFeed(&parser,byte,ms);
  }
  HWT101_ParserExpire(&parser,HAL_GetTick());
  RefreshDiagnostics();
}
uint8_t HWT101_Read(HWT101_Data *data)
{
  if(!data) return 0;
  HWT101_ParserExpire(&parser,HAL_GetTick());
  RefreshDiagnostics();
  *data=parser.data; return data->valid;
}
uint8_t HWT101_Zero(void)
{
  uint8_t result;
  HWT101_Poll();result=HWT101_ParserZero(&parser,HAL_GetTick());RefreshDiagnostics();return result;
}
uint32_t HWT101_RxBytes(void) { return bytes; }
uint8_t HWT101_Recover(uint32_t expected_epoch,uint32_t expected_sequence) {
 uint8_t result;HWT101_Poll();
 if(parser.data.discontinuity_sequence!=expected_epoch || parser.data.sequence!=expected_sequence ||
    !parser.data.valid || !parser.data.rate_valid ||
    (uint32_t)(HAL_GetTick()-parser.data.timestamp_ms)>=300U)return 0;
 result=HWT101_ParserRecover(&parser,HAL_GetTick());RefreshDiagnostics();return result;
}
uint32_t HWT101_RxErrors(void) { return errors; }
void HWT101_GetDiagnostics(HWT101_Diagnostics *out) {
 uint32_t mask;if(!out)return;RefreshDiagnostics();mask=__get_PRIMASK();__disable_irq();
 *out=g_hwt101_diagnostics;__set_PRIMASK(mask);
}
