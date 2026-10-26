#ifndef SERVO_H
#define SERVO_H
#include "main.h"
#define SERVO_RESET_TENTHS 900U
HAL_StatusTypeDef Servo_Init(void);
void Servo_Hold(void);
void Servo_Disable(void);
uint16_t Servo_AngleToPulse(uint16_t tenths);
void Servo_SetTenths(uint16_t angle);
uint16_t Servo_CurrentTenths(void);
#endif
