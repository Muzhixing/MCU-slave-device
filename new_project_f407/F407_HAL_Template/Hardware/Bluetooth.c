#include "Bluetooth.h"
#include "UpperPacket.h"
#include "SystemControl.h"
static UART_HandleTypeDef uart;
static uint8_t rx_byte,tx_frame[20];
typedef struct {uint8_t byte;uint32_t ms;} RxEntry;
static volatile RxEntry queue[256];
static volatile uint32_t head,tail,rx_fault;
static volatile uint8_t tx_busy;
static UpperParser parser;
static uint8_t sequence,result,dirty;
static uint32_t last_tx;
volatile BluetoothStatus g_bluetooth;
HAL_StatusTypeDef Bluetooth_Init(void)
{
  GPIO_InitTypeDef gpio={0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  gpio.Pin=GPIO_PIN_10|GPIO_PIN_11;
  gpio.Mode=GPIO_MODE_AF_PP; gpio.Pull=GPIO_PULLUP;
  gpio.Speed=GPIO_SPEED_FREQ_HIGH; gpio.Alternate=GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC,&gpio);
  uart.Instance=USART3;
  uart.Init.BaudRate=BLUETOOTH_BAUD_RATE;
  uart.Init.WordLength=UART_WORDLENGTH_8B;
  uart.Init.StopBits=UART_STOPBITS_1;
  uart.Init.Parity=UART_PARITY_NONE;
  uart.Init.Mode=UART_MODE_TX_RX;
  uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  uart.Init.OverSampling=UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&uart)!=HAL_OK) return HAL_ERROR;
  head=tail=rx_fault=0;tx_busy=0;sequence=result=0;last_tx=HAL_GetTick();dirty=1;UpperPacket_Init(&parser);
  g_bluetooth.connected=0;


  HAL_NVIC_SetPriority(USART3_IRQn,3,0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  return HAL_UART_Receive_IT(&uart,&rx_byte,1);
}


void USART3_IRQHandler(void){HAL_UART_IRQHandler(&uart);}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h){
 uint32_t next;if(h!=&uart)return;
 next=(head+1U)&255U;
 if(next==tail)rx_fault=1;
 else{queue[head].byte=rx_byte;queue[head].ms=HAL_GetTick();head=next;}
 if(HAL_UART_Receive_IT(&uart,&rx_byte,1)!=HAL_OK)rx_fault=1;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h){if(h==&uart)tx_busy=0;}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h){if(h==&uart)rx_fault=1;}
void Bluetooth_Update(void){
 uint32_t budget=128,now=HAL_GetTick();UpperCommand cmd;
 if(rx_fault){
  uint32_t mask=__get_PRIMASK();__disable_irq();tail=head;rx_fault=0;__set_PRIMASK(mask);
  g_bluetooth.rx_errors++;UpperPacket_Init(&parser);SystemControl_CommsFault();result=6;dirty=1;
  (void)HAL_UART_AbortReceive(&uart);(void)HAL_UART_Receive_IT(&uart,&rx_byte,1);
 }
 while(tail!=head && budget--){
  uint8_t byte=queue[tail].byte;uint32_t ms=queue[tail].ms;
  int parsed;tail=(tail+1U)&255U;
  parsed=UpperPacket_Feed(&parser,byte,ms,&cmd);
  if(parsed==1){sequence++;g_bluetooth.valid_packets++;result=SystemControl_Apply(&cmd);dirty=1;}
  else if(parsed<0){g_bluetooth.invalid_packets++;result=1;dirty=1;}
 }
 /* One persistent buffer, nonblocking UART IRQ transmit. Latest status
  * every250ms, command changes at most50Hz; not one ACK per queued packet. */
 if(!tx_busy && ((dirty && (uint32_t)(now-last_tx)>=20U)||(uint32_t)(now-last_tx)>=250U)){
  SystemControl_Status(tx_frame,sequence,result,(uint8_t)g_bluetooth.rx_errors);
  tx_busy=1;
  if(HAL_UART_Transmit_IT(&uart,tx_frame,sizeof(tx_frame))!=HAL_OK){tx_busy=0;SystemControl_CommsFault();result=6;}
  else{last_tx=now;dirty=0;}
 }
}
