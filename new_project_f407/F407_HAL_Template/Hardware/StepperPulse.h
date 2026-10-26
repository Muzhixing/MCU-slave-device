#ifndef STEPPER_PULSE_H
#define STEPPER_PULSE_H
#include "stm32f4xx_hal.h"
HAL_StatusTypeDef StepperPulse_Init(void);
HAL_StatusTypeDef StepperPulse_Start(unsigned axis, unsigned reverse, uint32_t pulses);
void StepperPulse_StopAll(void);
uint8_t StepperPulse_Busy(void);
int32_t StepperPulse_Position(unsigned axis);
uint8_t StepperPulse_Zero(void);
uint8_t StepperPulse_AxisBusy(unsigned axis);
uint8_t StepperPulse_ZeroAxis(unsigned axis);
#endif
