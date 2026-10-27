#include "AxisControl.h"
#include "StepperPulse.h"
int32_t AxisControl_TargetPulses(unsigned a,uint8_t mm){
 int32_t n;if(mm>175)mm=175;
 if(a==0)return (int32_t)mm*800;
 n=((int32_t)mm*2060+40)/81;return n>4450?4450:n;
}
uint16_t AxisControl_PositionTenths(unsigned a){
 int32_t n=StepperPulse_Position(a);
 if(n<0)n=0;
 return a==0?(uint16_t)((n+40)/80):(uint16_t)((n*81+103)/206);
}
HAL_StatusTypeDef AxisControl_MoveTo(unsigned a,uint8_t mm){
 int32_t delta;
 if(a>1)return HAL_ERROR;
 if(StepperPulse_Busy())return HAL_BUSY;
 delta=AxisControl_TargetPulses(a,mm)-StepperPulse_Position(a);
 if(!delta)return HAL_OK;
 return StepperPulse_Start(a,delta<0,(uint32_t)(delta<0?-delta:delta));
}
