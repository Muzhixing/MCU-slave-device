#include <assert.h>
#include <stdio.h>
#include "AxisFeedback.h"
#include "DriverCAN.h"
static void cycle(unsigned a,uint32_t t){
 AxisFeedback_Receive(a,7,t);assert(!AxisFeedback_Verified(a,t));
 AxisFeedback_Receive(a,3,t+20);AxisFeedback_Receive(a,3,t+120);
 assert(!AxisFeedback_Verified(a,t+120));AxisFeedback_Receive(a,3,t+220);
 assert(AxisFeedback_Verified(a,t+220));
}
int main(void){
 AxisFeedbackState s;unsigned a=9;uint8_t flags=0,d[]={0x3B,7,0x6B};
 assert(DriverCAN_Decode(0x100,1,0,d,3,&a,&flags)&&a==0&&flags==7);
 assert(DriverCAN_Decode(0x200,1,0,d,3,&a,&flags)&&a==1);
 assert(!DriverCAN_Decode(0x101,1,0,d,3,&a,&flags));
 assert(!DriverCAN_Decode(0x300,1,0,d,3,&a,&flags));
 assert(!DriverCAN_Decode(0x100,0,0,d,3,&a,&flags));
 assert(!DriverCAN_Decode(0x100,1,1,d,3,&a,&flags));
 assert(!DriverCAN_Decode(0x100,1,0,d,2,&a,&flags));
 d[2]=0;assert(!DriverCAN_Decode(0x100,1,0,d,3,&a,&flags));d[2]=0x6B;
 d[0]=0x3A;assert(!DriverCAN_Decode(0x100,1,0,d,3,&a,&flags));d[0]=0x3B;
 d[1]=0x83;assert(!DriverCAN_Decode(0x100,1,0,d,3,&a,&flags));
 AxisFeedback_Init(0);
 for(uint32_t t=0;t<2000;t+=20)AxisFeedback_Receive(0,3,t);
 assert(!AxisFeedback_Verified(0,1980)); /* Idle alone never creates an origin. */
 cycle(0,2100);assert(!AxisFeedback_Verified(1,2320));cycle(1,2330);
 assert(AxisFeedback_Verified(0,2550)&&AxisFeedback_Verified(1,2550));
 AxisFeedback_Update(2820);assert(!AxisFeedback_Verified(0,2820));
 AxisFeedback_Receive(0,3,2830);assert(!AxisFeedback_Verified(0,2830)); /* reconnect isn't rehome */
 cycle(0,2900);AxisFeedback_Receive(0,11,3140);assert(!AxisFeedback_Verified(0,3140));
 AxisFeedback_Get(0,&s);assert(s.phase==AXIS_HOME_FAILED);
 AxisFeedback_Receive(0,3,3160);assert(!AxisFeedback_Verified(0,3160));
 cycle(0,3200);AxisFeedback_Receive(0,0,3440);assert(!AxisFeedback_Verified(0,3440));
 cycle(0,3500);AxisFeedback_Init(3800);AxisFeedback_Receive(0,3,3810);
 assert(!AxisFeedback_Verified(0,3810)); /* MCU-only reset discards witness. */
 AxisFeedback_Init(0);
 for(uint32_t t=0;t<=30200;t+=100)AxisFeedback_Receive(0,7,t);
 AxisFeedback_Get(0,&s);assert(s.phase==AXIS_HOME_TIMEOUT&&!s.verified);
 AxisFeedback_Receive(0,3,30300);assert(!AxisFeedback_Verified(0,30300));cycle(0,30400);
 AxisFeedback_Init(0xFFFFFF00U);cycle(0,0xFFFFFF20U);
 assert(AxisFeedback_Verified(0,0xFFFFFFFCU));AxisFeedback_Update(0x200U);
 assert(!AxisFeedback_Verified(0,0x200U));
 /* A gap between busy and idle invalidates the cycle. */
 AxisFeedback_Init(0);AxisFeedback_Receive(0,7,0);AxisFeedback_Receive(0,3,600);
 assert(!AxisFeedback_Verified(0,600));
 puts("PASS CAN exact address/frame/function/checksum, independent observed home cycles, first-idle rejection, stale/reconnect, failure, MCU reset, timeout, wraparound");
}
