#ifndef WATER_PUMP_H
#define WATER_PUMP_H
#include "main.h"
/* Pump is excluded from integrated operation; only force-off is retained. */
void Pump_Init(void);
void Pump_Off(void);
#endif
