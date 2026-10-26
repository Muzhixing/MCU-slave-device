#ifndef MAIN_H
#define MAIN_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
void SystemClock_Config(void);
void Error_Handler(void);
#ifdef __cplusplus
}
#endif
#endif
