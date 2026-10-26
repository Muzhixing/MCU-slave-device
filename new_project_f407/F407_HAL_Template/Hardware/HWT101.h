#ifndef HWT101_H
#define HWT101_H
#include "main.h"
#include "HWT101_Parser.h"
/* Read-only USART2 RX: PA3 <- sensor TX, 115200 8N1. PA2 untouched. */
HAL_StatusTypeDef HWT101_Init(void);
void HWT101_Poll(void);
uint8_t HWT101_Read(HWT101_Data *data);
uint8_t HWT101_Zero(void);
uint32_t HWT101_RxBytes(void);
uint32_t HWT101_RxErrors(void);
#endif
