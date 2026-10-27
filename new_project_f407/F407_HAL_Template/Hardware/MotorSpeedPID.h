/* Ported from MCU-slave-device PIDtest c433766c; incremental speed PID. */
#ifndef MOTOR_SPEED_PID_H
#define MOTOR_SPEED_PID_H

#include <stdint.h>

#define MOTOR_SPEED_PID_MOTOR_COUNT 4U
#define MOTOR_SPEED_PID_DEFAULT_KP  2
#define MOTOR_SPEED_PID_DEFAULT_KI  1
#define MOTOR_SPEED_PID_DEFAULT_KD  0
#define MOTOR_SPEED_PID_OUTPUT_MAX  300 /* Initial F407 bench limit: 30%. */
#define MOTOR_SPEED_PID_PERIOD_MS   10U

void MotorSpeedPID_Init(void);
void MotorSpeedPID_Stop(void);
/* RunStep: 0=not due, 1=updated, 2=driver error and all stopped. */
void MotorSpeedPID_SetTargets(const int32_t targets[MOTOR_SPEED_PID_MOTOR_COUNT]);
void MotorSpeedPID_SetMeasuredSpeeds(
    const int32_t measured_speeds[MOTOR_SPEED_PID_MOTOR_COUNT]);
void MotorSpeedPID_SetParameters(uint8_t motor_index,
                                 int32_t kp,
                                 int32_t ki,
                                 int32_t kd,
                                 int32_t gain_divisor,
                                 int32_t feedforward_pwm);
uint8_t MotorSpeedPID_RunStep(void);
void Motor_A_SetTargetSpeed(int32_t speed);
void Motor_B_SetTargetSpeed(int32_t speed);
void Motor_C_SetTargetSpeed(int32_t speed);
void Motor_D_SetTargetSpeed(int32_t speed);
int32_t MotorSpeedPID_GetTarget(uint8_t motor_index);
int32_t MotorSpeedPID_GetMeasuredSpeed(uint8_t motor_index);
int32_t MotorSpeedPID_GetOutput(uint8_t motor_index);

#endif
