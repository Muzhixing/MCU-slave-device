#include "MotorControl.h"
volatile MotorControlStatus g_motor_control;
static uint32_t last_tick, no_feedback[4], reverse_feedback[4];

void MotorControl_Stop(void) { MotorSpeedPID_Stop(); }

static void Fault(uint32_t code, uint32_t wheel)
{
  uint32_t i;
  g_motor_control.fault = code; g_motor_control.fault_motor = wheel;
  MotorControl_Stop();
  for(i=0; i<4; ++i) { g_motor_control.pwm[i]=0; g_motor_control.target[i]=0; }
}

HAL_StatusTypeDef MotorControl_Init(void)
{
  uint32_t i;
  if(MotorEncoder_Init()!=HAL_OK) { Motor_StopAll(); return HAL_ERROR; }
  MotorSpeedPID_Init();
  /* Original PIDtest tuned gains / 100; must be retuned on F407 hardware. */
  for(i=0; i<4; ++i) {
    MotorSpeedPID_SetParameters((uint8_t)i, i==1 ? 23 : 30, 20, 0, 100, 0);
    no_feedback[i]=reverse_feedback[i]=0;
    g_motor_control.pulses[i]=g_motor_control.speed[i]=0;
    g_motor_control.target[i]=g_motor_control.pwm[i]=0;
  }
  g_motor_control.fault=0; g_motor_control.fault_motor=4;
  last_tick=HAL_GetTick();
  return HAL_OK;
}

void MotorControl_Update(void)
{
  int32_t pulses[4], speed[4];
  uint32_t now=HAL_GetTick(), elapsed=now-last_tick, i;
  if(g_motor_control.fault) { MotorControl_Stop(); return; }
  if(elapsed<10U) { return; }
  last_tick=now;
  if(elapsed>20U) { Fault(3,4); return; }
  MotorEncoder_Sample(elapsed,pulses,speed);
  for(i=0; i<4; ++i) {
    int32_t target=MotorSpeedPID_GetTarget((uint8_t)i);
    int32_t pwm=MotorSpeedPID_GetOutput((uint8_t)i);
    g_motor_control.pulses[i]=pulses[i]; g_motor_control.speed[i]=speed[i];
    g_motor_control.target[i]=target;
    if(target && pwm) {
      no_feedback[i]=pulses[i] ? 0 : no_feedback[i]+elapsed;
      reverse_feedback[i]=((target>0 && pulses[i]<0)||(target<0 && pulses[i]>0)) ? reverse_feedback[i]+elapsed : 0;
      if(no_feedback[i]>=500U) { Fault(1,i); return; }
      if(reverse_feedback[i]>=30U) { Fault(2,i); return; }
    } else { no_feedback[i]=reverse_feedback[i]=0; }
  }
  MotorSpeedPID_SetMeasuredSpeeds(speed);
  if(MotorSpeedPID_RunStep()==2U) { Fault(4,4); return; }
  for(i=0; i<4; ++i) { g_motor_control.pwm[i]=MotorSpeedPID_GetOutput((uint8_t)i); }
}
