#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H
#include "motor.h"
#define MOTOR_ENCODER_RESOLUTION 11U
#define MOTOR_ENCODER_GEAR_RATIO 15U
/* Legacy PIDtest speed units, NOT calibrated wheel RPM. */
#define MOTOR_ENCODER_SPEED_SCALE 2U
/* A verified on board; B/C/D inverted per reported four-wheel feedback test. */
#define MOTOR_ENCODER_SIGN_A -1
#define MOTOR_ENCODER_SIGN_B -1
#define MOTOR_ENCODER_SIGN_C 1
#define MOTOR_ENCODER_SIGN_D 1
HAL_StatusTypeDef MotorEncoder_Init(void);
uint16_t MotorEncoder_ReadRaw(MotorId id);
void MotorEncoder_Sample(uint32_t elapsed_ms, int32_t pulses[4], int32_t speeds[4]);
int32_t MotorEncoder_Delta(uint16_t previous, uint16_t current);
int32_t MotorEncoder_PulseToSpeed(int32_t pulses, uint32_t elapsed_ms);
#endif
