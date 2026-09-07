/**
 ******************************************************************************
 * @file    hcan.h
 * @pin_resources PA11=CAN1_RX and PA12=CAN1_TX through an external transceiver.
 * @function Declares the original CAN start and command-send interfaces.
 * @purpose Connects Emm_V5 commands to the STM32 CAN1 peripheral.
 ******************************************************************************
 */

#ifndef __HCAN_H__
#define __HCAN_H__

#include "stm32f1xx_hal.h"
#include "can.h"

#define	CAN_NUM   &hcan

extern volatile HAL_StatusTypeDef can_filter_status;
extern volatile HAL_StatusTypeDef can_notify_status;
extern volatile HAL_StatusTypeDef can_start_status;
extern volatile HAL_StatusTypeDef can_tx_status;
extern volatile uint32_t can_tx_mailbox;
extern volatile uint8_t can_rx_received;
extern CAN_RxHeaderTypeDef can_rx_header;
extern uint8_t can_rx_data[8];


void CAN_Start(CAN_HandleTypeDef *hcan);
void can_SendCmd(uint8_t *cmd, uint8_t len);
void CAN_SendEXData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
void CAN_SendData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
void CAN_Rx_Callback(CAN_HandleTypeDef *hcan);








#endif
