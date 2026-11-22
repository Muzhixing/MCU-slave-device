/* Actual parser + controller + pulse engine; only peripherals are mocked. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "SystemControl.h"
#include "StepperPulse.h"
#include "AxisControl.h"
#include "MotorControl.h"
#include "HWT101.h"
#include "ultrasound.h"
GPIO_TypeDef ga,gd,ge;TIM_TypeDef t7;uint32_t tick_ms;
MotorControlStatus g_motor_control;
static HWT101_Parser parser;
static int streaming=1, rotating=0;
static uint16_t angle_word=8000,servo=900;
static int32_t wheels[4];
static unsigned recovery_calls;
void MotorControl_Stop(void){memset(wheels,0,sizeof(wheels));}
void MotorSpeedPID_SetTargets(const int32_t *v){memcpy(wheels,v,sizeof(wheels));}
void Servo_Hold(void){}
void Servo_SetTenths(uint16_t a){servo=a;}
uint16_t Servo_CurrentTenths(void){return servo;}
uint8_t HWT101_Read(HWT101_Data*d){HWT101_ParserExpire(&parser,tick_ms);*d=parser.data;return d->valid;}
uint8_t HWT101_Recover(uint32_t epoch,uint32_t sequence){
 if(epoch!=parser.data.discontinuity_sequence || sequence!=parser.data.sequence)return 0;
 recovery_calls++;return HWT101_ParserRecover(&parser,tick_ms);
}
void Ultrasound_Update(void){}
bool Ultrasound_TakeMeasurement(uint16_t*d,bool*v){*d=0;*v=false;return true;}
void Pump_Off(void){}
void TIM7_IRQHandler(void);
static void frame(void){unsigned char f[11]={0x55,0x53,0,0,0,0,0,0,0,0,0};
 if(rotating)angle_word+=100;f[6]=(uint8_t)angle_word;f[7]=(uint8_t)(angle_word>>8);
 for(int i=0;i<10;i++)f[10]+=f[i];for(int i=0;i<11;i++)HWT101_ParserFeed(&parser,f[i],tick_ms);
}
static void ms(unsigned n){while(n--){tick_ms++;if(streaming && tick_ms%20==0)frame();
 for(unsigned i=0;i<8;i++){t7.SR=1;TIM7_IRQHandler();}SystemControl_Update();}}
static int idle(void){return !wheels[0]&&!wheels[1]&&!wheels[2]&&!wheels[3]&&!StepperPulse_Busy();}
static void status(uint8_t*out){SystemControl_Status(out,101,0,0);uint8_t sum=0;
 assert(out[0]==0xB5&&out[1]==1&&out[19]==0xB6);for(unsigned i=1;i<18;i++)sum^=out[i];assert(out[18]==sum);}
static void preserved(int32_t b,int32_t c){uint8_t s[20];status(s);assert(!(s[5]&1)&&((s[5]&6)==6));
 assert(StepperPulse_Position(0)==b&&StepperPulse_Position(1)==c&&idle());}
static void initialize(void){tick_ms=0;streaming=1;rotating=0;recovery_calls=0;angle_word=8000;
 memset(&g_motor_control,0,sizeof(g_motor_control));assert(StepperPulse_Init()==HAL_OK);
 HWT101_ParserInit(&parser);SystemControl_Init();ms(1100);assert(g_heading_diagnostics.phase==HEADING_READY);
 UpperCommand z={0,0,0,90,0,0,6};assert(SystemControl_Apply(&z)==0);
}
static void recovering_step_fault(void){
 initialize();UpperCommand c={0,0,0,90,2,3,1};assert(SystemControl_Apply(&c)==0);ms(5000);
 int32_t b=StepperPulse_Position(0),s=StepperPulse_Position(1);assert(b==1600&&s==76);
 c.lift=30;assert(SystemControl_Apply(&c)==0);ms(650);assert(StepperPulse_Busy());
 HWT101_ParserBreak(&parser);SystemControl_Update();b=StepperPulse_Position(0);s=StepperPulse_Position(1);
 assert(b>1600);preserved(b,s);assert(SystemControl_Apply(&c)==4);
 c.flags=0;assert(SystemControl_Apply(&c)==0);c.flags=1; /* early stop is NOT a resume token */
 for(unsigned i=0;i<80;i++){ms(20);assert(SystemControl_Apply(&c)==3);preserved(b,s);}
 assert(g_heading_diagnostics.phase==HEADING_WAIT_ACK&&recovery_calls==1);
 assert(g_heading_diagnostics.losses==1&&g_heading_diagnostics.recoveries==1&&g_heading_diagnostics.rejected_enables>=80);
 uint8_t raw[20];status(raw);assert((raw[5]&8)&&(raw[5]&32));
 assert(recovery_calls==1);ms(2000);preserved(b,s);assert(recovery_calls==1);
 c.flags=0;assert(SystemControl_Apply(&c)==0);ms(100);preserved(b,s);
 c.flags=1;assert(SystemControl_Apply(&c)==0);ms(600);assert(StepperPulse_Busy());
 c.flags=0;assert(SystemControl_Apply(&c)==0);
 puts("PASS interrupted B/C pulse position and zero flags preserved; 80 old enables rejected; post-recovery stop+enable required");
}
static void recovery_noise_and_gap(void){
 initialize();UpperCommand c={30,0,0,100,0,0,1};assert(SystemControl_Apply(&c)==0);ms(600);assert(!idle());
 streaming=0;ms(600);assert(idle());assert(g_heading_diagnostics.phase==HEADING_WAIT_STABLE);
 c.flags=0;assert(SystemControl_Apply(&c)==0);uint16_t held=servo;
 streaming=1;rotating=1;ms(1600);assert(recovery_calls==0&&servo==held&&idle());
 rotating=0;ms(700);assert(recovery_calls==0);
 HWT101_ParserBreakReason(&parser,HWT101_BREAK_OVERFLOW);ms(700);assert(recovery_calls==0);
 ms(500);assert(recovery_calls==1&&g_heading_diagnostics.phase==HEADING_WAIT_ACK&&idle());
 /* Lose fresh samples between acknowledgement and the requested enable. */
 c.flags=0;assert(SystemControl_Apply(&c)==0);streaming=0;ms(350);c.flags=1;assert(SystemControl_Apply(&c)==3);
 streaming=1;ms(1150);assert(idle()&&g_heading_diagnostics.phase==HEADING_WAIT_ACK);
 c.flags=0;assert(SystemControl_Apply(&c)==0);c.flags=1;assert(SystemControl_Apply(&c)==0);ms(600);assert(!idle());
 c.flags=0;assert(SystemControl_Apply(&c)==0);
 puts("PASS timeout recovery, moving sensor rejection, repeated fault resets stable window, stale-after-ack interlock, servo hold");
}
static void startup_and_gyro_only(void){
 tick_ms=0;streaming=0;rotating=0;recovery_calls=0;memset(&g_motor_control,0,sizeof(g_motor_control));
 assert(StepperPulse_Init()==HAL_OK);HWT101_ParserInit(&parser);SystemControl_Init();
 HWT101_ParserBreak(&parser);ms(1200);assert(!recovery_calls);
 streaming=1;ms(1200);assert(recovery_calls==1&&g_heading_diagnostics.phase==HEADING_WAIT_ACK&&idle());
 HWT101_ParserBreak(&parser);streaming=0;ms(1);
 for(unsigned j=0;j<70;j++){uint8_t f[11]={0x55,0x52,0,0,0,0,0,0,0,0,0};
  for(unsigned i=0;i<10;i++)f[10]+=f[i];for(unsigned i=0;i<11;i++)HWT101_ParserFeed(&parser,f[i],tick_ms);ms(20);}
 assert(recovery_calls==1&&g_heading_diagnostics.phase==HEADING_WAIT_STABLE);
 puts("PASS startup receive fault recovery and gyro-only stream cannot recover yaw");
}
static void rollover(void){
 tick_ms=UINT32_MAX-600;streaming=1;rotating=0;recovery_calls=0;memset(&g_motor_control,0,sizeof(g_motor_control));
 assert(StepperPulse_Init()==HAL_OK);HWT101_ParserInit(&parser);SystemControl_Init();ms(1300);
 assert(g_heading_diagnostics.phase==HEADING_READY);
 HWT101_ParserBreak(&parser);ms(1200);assert(g_heading_diagnostics.phase==HEADING_WAIT_ACK&&idle());
 puts("PASS millisecond counter rollover");
}
static unsigned yaw_wire(void){uint8_t s[20];status(s);assert((s[5]&8)&&(s[5]&32));return (unsigned)s[10]|((unsigned)s[11]<<8);}
static void preserve_heading_reference(void){
 initialize();
 /* Establish a nonzero startup reference relative to the parser's first frame. */
 HWT101_ParserInit(&parser);SystemControl_Init();ms(40);angle_word+=364;ms(1100);
 assert(g_heading_diagnostics.phase==HEADING_READY);
 float startup_reference=parser.data.continuous_deg;assert(startup_reference>1.9f);
 angle_word+=2000;ms(100);unsigned before=yaw_wire();assert(before>100&&before<120);
 float zero=parser.zero_deg;HWT101_ParserBreak(&parser);ms(1200);
 assert(g_heading_diagnostics.phase==HEADING_WAIT_ACK&&yaw_wire()==before&&parser.zero_deg==zero);
 /* The raw heading changed during the gap: recover relative to the old zero,
    not relative to the new stationary orientation. */
 HWT101_ParserBreak(&parser);angle_word+=1000;ms(1200);
 assert(g_heading_diagnostics.phase==HEADING_WAIT_ACK&&yaw_wire()>before+50&&parser.zero_deg==zero&&idle());
 /* Crossing signed sensor wrap still keeps the original modulo heading. */
 angle_word=32586;ms(1200);HWT101_ParserBreak(&parser);angle_word=(uint16_t)(-32586);ms(1200);
 assert(g_heading_diagnostics.phase==HEADING_WAIT_ACK&&parser.zero_deg==zero&&idle());
 unsigned expected=(unsigned)((HWT101_AngleError((float)(-32586)*180.0f/32768.0f,zero)-startup_reference)*10.0f+0.5f);
 /* The result here is a positive relative angle below 180 degrees. */
 assert(yaw_wire()==expected);
 puts("PASS original yaw zero preserved, orientation changed during loss, signed-angle wrap, no implicit new zero");
}
int main(void){recovering_step_fault();recovery_noise_and_gap();startup_and_gyro_only();rollover();preserve_heading_reference();return 0;}
