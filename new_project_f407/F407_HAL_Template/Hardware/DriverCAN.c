#include "DriverCAN.h"
#include "AxisFeedback.h"
#include <string.h>
static CAN_HandleTypeDef bus;
static uint32_t last_query,tx_at;
static unsigned next_axis;
static uint8_t started,pending;
static uint32_t mailbox;
volatile DriverCANDiagnostics g_driver_can;
HAL_StatusTypeDef DriverCAN_Init(void){
 GPIO_InitTypeDef pins={0};CAN_FilterTypeDef filter={0};
 memset((void *)&g_driver_can,0,sizeof(g_driver_can));g_driver_can.version=1;
 AxisFeedback_Init(HAL_GetTick());started=pending=0;next_axis=0;
 if(HAL_RCC_GetPCLK1Freq()!=42000000U)return HAL_ERROR;
 __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_CAN1_CLK_ENABLE();
 pins.Pin=GPIO_PIN_11|GPIO_PIN_12;pins.Mode=GPIO_MODE_AF_PP;
 pins.Pull=GPIO_PULLUP;pins.Speed=GPIO_SPEED_FREQ_VERY_HIGH;pins.Alternate=GPIO_AF9_CAN1;
 HAL_GPIO_Init(GPIOA,&pins);
 bus.Instance=CAN1;bus.Init.Prescaler=6;bus.Init.Mode=CAN_MODE_NORMAL;
 bus.Init.SyncJumpWidth=CAN_SJW_1TQ;bus.Init.TimeSeg1=CAN_BS1_11TQ;bus.Init.TimeSeg2=CAN_BS2_2TQ;
 bus.Init.TimeTriggeredMode=DISABLE;bus.Init.AutoBusOff=ENABLE;bus.Init.AutoWakeUp=DISABLE;
 bus.Init.AutoRetransmission=DISABLE;bus.Init.ReceiveFifoLocked=ENABLE;bus.Init.TransmitFifoPriority=ENABLE;
 /* 42MHz / (6 * (1+11+2)) = 500kbit/s. */
 if(HAL_CAN_Init(&bus)!=HAL_OK)return HAL_ERROR;
 filter.FilterBank=0;filter.FilterMode=CAN_FILTERMODE_IDLIST;filter.FilterScale=CAN_FILTERSCALE_32BIT;
 filter.FilterIdHigh=((DRIVER_B_ADDRESS<<8)<<3)>>16;
 filter.FilterIdLow=(((DRIVER_B_ADDRESS<<8)<<3)|4U)&0xFFFFU;
 filter.FilterMaskIdHigh=((DRIVER_C_ADDRESS<<8)<<3)>>16;
 filter.FilterMaskIdLow=(((DRIVER_C_ADDRESS<<8)<<3)|4U)&0xFFFFU;
 filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;filter.FilterActivation=ENABLE;filter.SlaveStartFilterBank=14;
 if(HAL_CAN_ConfigFilter(&bus,&filter)!=HAL_OK || HAL_CAN_Start(&bus)!=HAL_OK)return HAL_ERROR;
 last_query=HAL_GetTick()-20U;started=1;return HAL_OK;
}
void DriverCAN_Poll(void){
 uint32_t now=HAL_GetTick();unsigned budget=8,a;uint8_t flags,d[8];CAN_RxHeaderTypeDef rx;
 CAN_TxHeaderTypeDef tx={0};
 AxisFeedback_Update(now);
 if(!started)return;
 if(bus.Instance->ESR&CAN_ESR_BOFF){
  g_driver_can.bus_faults++;g_driver_can.hal_error=bus.Instance->ESR;
  AxisFeedback_Invalidate(0,now);AxisFeedback_Invalidate(1,now);
  while(budget-- && HAL_CAN_GetRxFifoFillLevel(&bus,CAN_RX_FIFO0))
   if(HAL_CAN_GetRxMessage(&bus,CAN_RX_FIFO0,&rx,d)!=HAL_OK)break;
  return;
 }
 if(bus.Instance->RF0R&CAN_RF0R_FOVR0){
  __HAL_CAN_CLEAR_FLAG(&bus,CAN_FLAG_FOV0);g_driver_can.rejected++;
  AxisFeedback_Invalidate(0,now);AxisFeedback_Invalidate(1,now);
  while(budget-- && HAL_CAN_GetRxFifoFillLevel(&bus,CAN_RX_FIFO0))
   if(HAL_CAN_GetRxMessage(&bus,CAN_RX_FIFO0,&rx,d)!=HAL_OK)break;
  return;
 }
 while(budget-- && HAL_CAN_GetRxFifoFillLevel(&bus,CAN_RX_FIFO0)){
  if(HAL_CAN_GetRxMessage(&bus,CAN_RX_FIFO0,&rx,d)!=HAL_OK)break;
  if(DriverCAN_Decode(rx.ExtId,rx.IDE==CAN_ID_EXT,rx.RTR==CAN_RTR_REMOTE,d,(uint8_t)rx.DLC,&a,&flags)){
   AxisFeedback_Receive(a,flags,now);g_driver_can.rx++;
  }else g_driver_can.rejected++;
 }
 if(pending){
  if(!HAL_CAN_IsTxMessagePending(&bus,mailbox))pending=0;
  else if((uint32_t)(now-tx_at)>=30U){HAL_CAN_AbortTxRequest(&bus,mailbox);pending=0;g_driver_can.tx_failures++;}
 }
 if(pending || (uint32_t)(now-last_query)<20U)return;
 last_query=now;tx.ExtId=(next_axis?DRIVER_C_ADDRESS:DRIVER_B_ADDRESS)<<8;
 tx.IDE=CAN_ID_EXT;tx.RTR=CAN_RTR_DATA;tx.DLC=2;tx.TransmitGlobalTime=DISABLE;
 /* The only outbound driver command: read homing flags. */
 d[0]=0x3B;d[1]=0x6B;
 if(HAL_CAN_AddTxMessage(&bus,&tx,d,&mailbox)==HAL_OK){pending=1;tx_at=now;g_driver_can.tx++;}
 else{g_driver_can.tx_failures++;g_driver_can.hal_error=HAL_CAN_GetError(&bus);}
 next_axis^=1U;
}
