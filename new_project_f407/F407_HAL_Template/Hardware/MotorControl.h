#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "MotorEncoder.h"
#include "MotorSpeedPID.h"
typedef struct {
  int32_t pulses[4], speed[4], target[4], pwm[4];
  uint32_t fault; /* 0=OK, 1=missing feedback, 2=reverse feedback, 3=late, 4=driver */
  uint32_t fault_motor; /* 0..3; 4=system */
} MotorControlStatus;
extern volatile MotorControlStatus g_motor_control;
HAL_StatusTypeDef MotorControl_Init(void);
void MotorControl_Update(void);
void MotorControl_Stop(void);
#endif
