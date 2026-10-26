#ifndef AXIS_CONTROL_H
#define AXIS_CONTROL_H
#include "stm32f4xx_hal.h"
int32_t AxisControl_TargetPulses(unsigned axis,uint8_t mm);
uint16_t AxisControl_PositionTenths(unsigned axis);
HAL_StatusTypeDef AxisControl_MoveTo(unsigned axis,uint8_t mm);
#endif
