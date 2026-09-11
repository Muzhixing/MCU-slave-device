/**
 ******************************************************************************
 * @file    hcan.c
 * @pin_resources PA11=CAN1_RX and PA12=CAN1_TX through default CAN1 in can.c.
 * @function Starts CAN1 and sends the original ZDT extended-frame packets.
 * @purpose Preserves the original driver transport and packet segmentation.
 ******************************************************************************
 */

#include "hcan.h"
#include <string.h>

#define CAN_SEND_TIMEOUT 10

volatile HAL_StatusTypeDef can_filter_status = HAL_ERROR;
volatile HAL_StatusTypeDef can_notify_status = HAL_ERROR;
volatile HAL_StatusTypeDef can_start_status = HAL_ERROR;
volatile HAL_StatusTypeDef can_tx_status = HAL_ERROR;
volatile uint32_t can_tx_mailbox = 0U;
volatile uint8_t can_rx_received = 0U;
CAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data[8] = {0U};

static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig)
{
    sFilterConfig->FilterIdHigh = 0;
    sFilterConfig->FilterIdLow = 0;
    sFilterConfig->FilterMaskIdHigh = 0;
    sFilterConfig->FilterMaskIdLow = 0;
    sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig->FilterBank = 0;
    sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig->FilterActivation = ENABLE;
    sFilterConfig->SlaveStartFilterBank = 0;
}

void CAN_Rx_Callback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                             &can_rx_header, can_rx_data) == HAL_OK)
    {
        can_rx_received = 1U;
    }
}

void CAN_Start(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    CAN_Filter_ParamsInit(&sFilterConfig);
    can_filter_status = HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    can_notify_status = HAL_CAN_ActivateNotification(
        hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    can_start_status = HAL_CAN_Start(hcan);
}

/* Preserve the original fixed-0x6B command framing and segmentation. */
void can_SendCmd(uint8_t *cmd, uint8_t len)
{
    uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint32_t mailbox;
    uint32_t sendTick;

    if(cmd == NULL || len < 2) return;

    // 轻量重置，不影响发送
    HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
    j = len - 2;

    while(i < j)
    {
        k = j - i;
        memset(TxData, 0, 8);

        // 帧头配置
        TxHeader.StdId = 0x00;
        TxHeader.ExtId = ((uint32_t)cmd[0] << 8) | (packNum & 0xFF);
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.TransmitGlobalTime = DISABLE;

        // 你的原始分包逻辑（完全匹配电机）
        TxData[0] = cmd[1];
        if(k < 7)
        {
            for(l=0; l<k; l++,i++) TxData[l+1] = cmd[i+2];
            TxHeader.DLC = k + 1;
        }
        else
        {
            for(l=0; l<7; l++,i++) TxData[l+1] = cmd[i+2];
            TxHeader.DLC = 8;
        }

        /* Queue the frame and retain HAL status/mailbox for diagnosis. */
        sendTick = HAL_GetTick();
        do
        {
            can_tx_status = HAL_CAN_AddTxMessage(
                &hcan, &TxHeader, TxData, &mailbox);
            if (can_tx_status == HAL_OK)
            {
                can_tx_mailbox = mailbox;
                break;
            }
            if(HAL_GetTick()-sendTick > CAN_SEND_TIMEOUT) break;
            HAL_Delay(1);
        } while (1);
        packNum++;
    }
}
