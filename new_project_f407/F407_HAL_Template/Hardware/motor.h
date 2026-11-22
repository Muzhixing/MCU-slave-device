#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"
typedef enum { MOTOR_LF = 0, MOTOR_LB, MOTOR_RF, MOTOR_RB, MOTOR_COUNT } MotorId;
HAL_StatusTypeDef Motor_Init(void);
/* Signed duty in per-mille, clamped to +/-999. A/B polarity is inverted
 * relative to C/D for wheel mounting. Stop before changing direction. */
HAL_StatusTypeDef Motor_Set(MotorId id, int32_t duty);
void Motor_StopAll(void);
#endif
