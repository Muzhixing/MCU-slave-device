#ifndef CAN_TEST_MAIN_H
#define CAN_TEST_MAIN_H
#include <stdint.h>
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY} HAL_StatusTypeDef;
typedef struct {uint32_t ESR,RF0R;} CAN_TypeDef;
extern CAN_TypeDef can_hw;
#define CAN1 (&can_hw)
#define GPIOA ((void *)0)
#define GPIO_PIN_11 (1U<<11)
#define GPIO_PIN_12 (1U<<12)
#define GPIO_MODE_AF_PP 2
#define GPIO_PULLUP 1
#define GPIO_SPEED_FREQ_VERY_HIGH 3
#define GPIO_AF9_CAN1 9
#define CAN_MODE_NORMAL 0
#define CAN_SJW_1TQ 0
#define CAN_BS1_11TQ 10
#define CAN_BS2_2TQ 1
#define DISABLE 0
#define ENABLE 1
#define CAN_FILTERMODE_IDLIST 1
#define CAN_FILTERSCALE_32BIT 1
#define CAN_FILTER_FIFO0 0
#define CAN_RX_FIFO0 0
#define CAN_ID_EXT 4
#define CAN_RTR_REMOTE 2
#define CAN_RTR_DATA 0
#define CAN_ESR_BOFF 4
#define CAN_RF0R_FOVR0 16
#define CAN_FLAG_FOV0 16
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_CAN1_CLK_ENABLE() ((void)0)
#define __HAL_CAN_CLEAR_FLAG(b,f) ((b)->Instance->RF0R &= ~(f))
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
typedef struct {uint32_t Prescaler,Mode,SyncJumpWidth,TimeSeg1,TimeSeg2,TimeTriggeredMode,AutoBusOff,AutoWakeUp,AutoRetransmission,ReceiveFifoLocked,TransmitFifoPriority;} CAN_InitTypeDef;
typedef struct {CAN_TypeDef *Instance;CAN_InitTypeDef Init;} CAN_HandleTypeDef;
typedef struct {uint32_t FilterBank,FilterMode,FilterScale,FilterIdHigh,FilterIdLow,FilterMaskIdHigh,FilterMaskIdLow,FilterFIFOAssignment,FilterActivation,SlaveStartFilterBank;} CAN_FilterTypeDef;
typedef struct {uint32_t ExtId,IDE,RTR,DLC,TransmitGlobalTime;} CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
void HAL_GPIO_Init(void *,GPIO_InitTypeDef *);
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *,CAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *,uint32_t,CAN_RxHeaderTypeDef *,uint8_t *);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *,CAN_TxHeaderTypeDef *,uint8_t *,uint32_t *);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *);
#endif
