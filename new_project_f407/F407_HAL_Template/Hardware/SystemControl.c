#include "SystemControl.h"
#include "MotorControl.h"
#include "StepperPulse.h"
#include "AxisControl.h"
#include "Servo.h"
#include "HWT101.h"
#include "ultrasound.h"
#include "water_pump.h"
#include <string.h>
#include <math.h>
/* State: 0 disabled, 1 ready/holding, 2 axes, 3 driving, 4 fault, 5 braking. */
static UpperCommand goal;
static uint8_t enabled,zeros,previous_flags,fault,state,referenced,stable,holding;
static uint32_t stable_since,stable_sequence,brake_since,control_tick,coast_since;
static uint8_t braking,coasting;
static int8_t wheel_sign[4];
static float reference,stable_yaw;
static HWT101_Data sensor;
static uint16_t distance;
static uint8_t distance_valid;
static float Wrap(float a){while(a>180.0f)a-=360.0f;while(a<=-180.0f)a+=360.0f;return a;}
static uint8_t Sensor(void){return HWT101_Read(&sensor)&&sensor.continuous_valid&&sensor.rate_valid&&(uint32_t)(HAL_GetTick()-sensor.timestamp_ms)<300U;}
static void Stop(void){
 enabled=0;StepperPulse_StopAll();MotorControl_Stop();Servo_Hold();Pump_Off();
 memset(wheel_sign,0,sizeof(wheel_sign));braking=0;holding=0;
 coasting=1;coast_since=HAL_GetTick();state=fault?4:0;
}
void SystemControl_Init(void){
 memset(&goal,0,sizeof(goal));zeros=previous_flags=fault=referenced=stable=0;
 distance_valid=0;control_tick=HAL_GetTick();Stop();
}
void SystemControl_CommsFault(void){fault=1;Stop();}
uint8_t SystemControl_Apply(const UpperCommand *c){
 uint8_t rising=(uint8_t)(c->flags & ~previous_flags & 6U);
 uint8_t was_enabled=enabled,was_busy=StepperPulse_Busy();
 if(!(c->flags&1U)){
  Stop();
  if(g_motor_control.fault){fault=2;state=4;return 4;}
  fault=0;state=0;
  if(rising && (was_enabled || was_busy))return 5;
  if(rising&2U){if(!StepperPulse_ZeroAxis(0))return 5;zeros|=1U;}
  if(rising&4U){if(!StepperPulse_ZeroAxis(1))return 5;zeros|=2U;}
  previous_flags=c->flags;return 0;
 }
 if(fault || g_motor_control.fault)return 4;
 if(rising)return 5; /* Zero only in a separate disabled frame. */
 if(zeros!=3U)return 2;
 if(!referenced || !Sensor())return 3;
 goal=*c;previous_flags=c->flags;enabled=1;Servo_SetTenths((uint16_t)c->servo*10U);
 return 0;
}
void SystemControl_Update(void){
 uint32_t now=HAL_GetTick();uint8_t valid=Sensor();bool measured_valid;
 uint16_t measured;unsigned a;int32_t desired[4],scale=1200;float error,turn;int moving=0,reverse=0;
 Ultrasound_Update();
 if(Ultrasound_TakeMeasurement(&measured,&measured_valid)){distance=measured;distance_valid=measured_valid?1:0;}
 if(!referenced){
  if(!valid || fabsf(sensor.yaw_rate_dps)>1.0f)stable=0;
  else if(sensor.sequence!=stable_sequence){
   stable_sequence=sensor.sequence;
   if(!stable || fabsf(sensor.continuous_deg-stable_yaw)>1.0f){stable=1;stable_since=now;stable_yaw=sensor.continuous_deg;}
   else if((uint32_t)(now-stable_since)>=1000U){reference=sensor.continuous_deg;referenced=1;}
  }
 }
 if(g_motor_control.fault){fault=2;Stop();return;}
 if(!enabled)return;
 if(!valid){fault=3;Stop();return;}
 /* Axes have priority over chassis. New absolute targets wait for the
  * current finite move to finish; latest packet replaces pending targets. */
 if(StepperPulse_Busy()){
  MotorControl_Stop();state=2;coasting=1;coast_since=now;return;
 }
 for(a=0;a<2;a++)if(StepperPulse_Position(a)!=AxisControl_TargetPulses(a,a?goal.slide:goal.lift)){
  if(!coasting){coasting=1;coast_since=now;}
  MotorControl_Stop();memset(wheel_sign,0,sizeof(wheel_sign));braking=0;holding=0;state=5;
  if((uint32_t)(now-coast_since)<500U)return;
  if(AxisControl_MoveTo(a,a?goal.slide:goal.lift)!=HAL_OK){fault=4;Stop();return;}
  state=2;return;
 }
 if(coasting){if((uint32_t)(now-coast_since)<500U){state=5;return;}coasting=0;}
 if((uint32_t)(now-control_tick)<10U)return;
 control_tick=now;
 error=Wrap((float)goal.heading-(sensor.continuous_deg-reference));
 if(holding){if(fabsf(error)>2.0f)holding=0;}
 else if(fabsf(error)<=1.0f && fabsf(sensor.yaw_rate_dps)<=3.0f)holding=1;
 turn=holding?0.0f:15.0f*error-2.0f*sensor.yaw_rate_dps;
 if(turn>600)turn=600;
 if(turn< -600)turn=-600;
 /* +yaw is left: A/B reverse, C/D forward. +lateral = left. */
 desired[0]=goal.forward*12-goal.lateral*12-(int32_t)turn;
 desired[1]=goal.forward*12+goal.lateral*12-(int32_t)turn;
 desired[2]=goal.forward*12+goal.lateral*12+(int32_t)turn;
 desired[3]=goal.forward*12-goal.lateral*12+(int32_t)turn;
 for(a=0;a<4;a++){int32_t v=desired[a]<0?-desired[a]:desired[a];if(v>scale)scale=v;}
 for(a=0;a<4;a++){
  desired[a]=(int32_t)((int64_t)desired[a]*1200/scale);
  if(desired[a]){moving=1;if(wheel_sign[a] && (desired[a]>0?1:-1)!=wheel_sign[a])reverse=1;}
 }
 if(braking){if((uint32_t)(now-brake_since)<500U){MotorControl_Stop();state=5;return;}braking=0;memset(wheel_sign,0,sizeof(wheel_sign));reverse=0;}
 if(reverse){MotorControl_Stop();braking=1;brake_since=now;state=5;return;}
 if(moving){MotorSpeedPID_SetTargets(desired);for(a=0;a<4;a++)if(desired[a])wheel_sign[a]=desired[a]>0?1:-1;state=3;}
 else{MotorControl_Stop();state=1;}
}
static void Put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
void SystemControl_Status(uint8_t o[20],uint8_t seq,uint8_t result,uint8_t errors){
 unsigned i;float yaw;uint8_t valid=Sensor();
 memset(o,0,20);o[0]=0xB5;o[1]=1;o[2]=seq;o[3]=result;o[4]=state;
 o[5]=(uint8_t)((enabled?1:0)|(zeros<<1)|(valid?8:0)|(distance_valid?16:0)|(referenced?32:0));
 Put16(o+6,AxisControl_PositionTenths(0));Put16(o+8,AxisControl_PositionTenths(1));
 if(valid&&referenced){yaw=Wrap(sensor.continuous_deg-reference);if(yaw<0)yaw+=360;Put16(o+10,(uint16_t)(yaw*10+0.5f));}
 else Put16(o+10,0xFFFF);
 Put16(o+12,distance_valid?distance:0xFFFF);
 o[14]=(uint8_t)(Servo_CurrentTenths()/10U);o[15]=fault;o[16]=(uint8_t)g_motor_control.fault;o[17]=errors;
 for(i=1;i<18;i++)o[18]^=o[i];
 o[19]=0xB6;
}
