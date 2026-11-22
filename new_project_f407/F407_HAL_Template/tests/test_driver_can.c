#include "DriverCAN.h"
#include "AxisFeedback.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
CAN_TypeDef can_hw;
static uint32_t now,clock_hz=42000000,queued,txcount,pending_tx,aborts,last_id;
static uint8_t rxflags=3;
uint32_t HAL_GetTick(void){return now;}
uint32_t HAL_RCC_GetPCLK1Freq(void){return clock_hz;}
void HAL_GPIO_Init(void *p,GPIO_InitTypeDef *g){(void)p;assert(g->Pin==(GPIO_PIN_11|GPIO_PIN_12)&&g->Alternate==9);}
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *b){assert(b->Init.Prescaler==6&&b->Init.TimeSeg1==10&&b->Init.TimeSeg2==1);assert(!b->Init.AutoRetransmission);return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *b,CAN_FilterTypeDef *f){(void)b;assert(f->FilterIdHigh==0&&f->FilterIdLow==0x804&&f->FilterMaskIdHigh==0&&f->FilterMaskIdLow==0x1004);return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *b){(void)b;return HAL_OK;}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *b,uint32_t f){(void)b;(void)f;return queued;}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *b,uint32_t f,CAN_RxHeaderTypeDef *r,uint8_t *d){(void)b;(void)f;assert(queued);queued--;memset(r,0,sizeof(*r));r->IDE=CAN_ID_EXT;r->ExtId=0x100;r->DLC=3;d[0]=0x3B;d[1]=rxflags;d[2]=0x6B;return HAL_OK;}
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *b,uint32_t m){(void)b;(void)m;return pending_tx;}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *b,uint32_t m){(void)b;(void)m;pending_tx=0;aborts++;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *b,CAN_TxHeaderTypeDef *t,uint8_t *d,uint32_t *m){(void)b;assert(t->IDE==CAN_ID_EXT&&t->RTR==CAN_RTR_DATA&&t->DLC==2);assert(t->ExtId==0x100||t->ExtId==0x200);assert(d[0]==0x3B&&d[1]==0x6B);txcount++;last_id=t->ExtId;*m=1;return HAL_OK;}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *b){(void)b;return 0;}
static void receive(uint8_t f,uint32_t t){now=t;queued=1;rxflags=f;DriverCAN_Poll();}
int main(void){
 assert(DriverCAN_Init()==HAL_OK);DriverCAN_Poll();assert(txcount==1&&last_id==0x100);
 now=19;DriverCAN_Poll();assert(txcount==1);now=20;DriverCAN_Poll();assert(txcount==2&&last_id==0x200);
 receive(3,40);assert(!AxisFeedback_Verified(0,now));
 receive(7,60);receive(3,80);receive(3,180);receive(3,280);assert(AxisFeedback_Verified(0,now));
 can_hw.RF0R=CAN_RF0R_FOVR0;receive(7,300);assert(!queued&&!AxisFeedback_Verified(0,now));
 receive(3,320);assert(!AxisFeedback_Verified(0,now));
 receive(7,340);receive(3,360);receive(3,460);receive(3,560);assert(AxisFeedback_Verified(0,now));
 can_hw.ESR=CAN_ESR_BOFF;{uint32_t before=txcount;receive(7,580);assert(txcount==before&&!queued&&!AxisFeedback_Verified(0,now));}
 can_hw.ESR=0;receive(3,600);assert(!AxisFeedback_Verified(0,now));
 pending_tx=1;now=610;DriverCAN_Poll();now=650;DriverCAN_Poll();assert(aborts>0);
 clock_hz=21000000;assert(DriverCAN_Init()==HAL_ERROR);{uint32_t before=txcount;now=1000;DriverCAN_Poll();assert(txcount==before&&!AxisFeedback_Verified(0,now));}
 puts("PASS actual CAN transport: PA11/12, bitrate/filter, read-only queries, queue fault discard, bus-off, timeout, init failure");
}
