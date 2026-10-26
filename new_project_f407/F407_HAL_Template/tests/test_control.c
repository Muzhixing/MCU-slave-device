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
static HWT101_Data data;static int sensor_valid=1;static int32_t wheels[4];static uint16_t servo=900;
void MotorControl_Stop(void){memset(wheels,0,sizeof(wheels));}
void MotorSpeedPID_SetTargets(const int32_t *v){memcpy(wheels,v,sizeof(wheels));}
void Servo_Hold(void){}
void Servo_SetTenths(uint16_t a){servo=a;}
uint16_t Servo_CurrentTenths(void){return servo;}
uint8_t HWT101_Read(HWT101_Data *d){*d=data;d->timestamp_ms=tick_ms;d->sequence=tick_ms;return sensor_valid;}
void Ultrasound_Update(void){}
bool Ultrasound_TakeMeasurement(uint16_t *d,bool *v){*d=234;*v=true;return true;}
void Pump_Off(void){}
void TIM7_IRQHandler(void);
static int idle_wheels(void){return !wheels[0]&&!wheels[1]&&!wheels[2]&&!wheels[3];}
static void ms(unsigned n){unsigned i;while(n--){tick_ms++;for(i=0;i<8;i++){t7.SR=1;TIM7_IRQHandler();}SystemControl_Update();}}
static void packet_tests(void){
 UpperParser p;UpperCommand c;unsigned i;int ret=0,count=0;
 uint8_t b[10]={0xB3,0x9C,100,180,180,179,255,0xFF,6,0xB4};
 UpperPacket_Init(&p);
 for(i=0;i<10;i++)ret=UpperPacket_Feed(&p,b[i],i,&c);
 assert(ret==1&&c.forward==-100&&c.lateral==100&&c.heading==360&&c.lift==175&&c.slide==175);
 /* Frame header in a legal payload and concatenated frames. */
 for(i=0;i<20;i++)if(UpperPacket_Feed(&p,b[i%10],20+i,&c)==1)count++;
 assert(count==2);b[1]=101;for(i=0;i<10;i++)ret=UpperPacket_Feed(&p,b[i],50+i,&c);assert(ret==-1);
 UpperPacket_Init(&p);b[1]=0;for(i=0;i<5;i++)UpperPacket_Feed(&p,b[i],i,&c);
 assert(UpperPacket_Feed(&p,0,100,&c)==0&&p.length==0);
 /* Noise, malformed footer, then valid frame must recover. */
 UpperPacket_Init(&p);UpperPacket_Feed(&p,12,1,&c);b[9]=0;
 for(i=0;i<10;i++)UpperPacket_Feed(&p,b[i],110+i,&c);
 b[9]=0xB4;for(i=0;i<10;i++)ret=UpperPacket_Feed(&p,b[i],130+i,&c);assert(ret==1);
 b[8]=8;for(i=0;i<10;i++)ret=UpperPacket_Feed(&p,b[i],160+i,&c);assert(ret==-1);
}
int main(void){
 UpperCommand c={0,0,0,90,0,0,1};uint8_t out[20],sum=0;unsigned i;int32_t n;
 packet_tests();assert(StepperPulse_Init()==HAL_OK);data.continuous_valid=data.rate_valid=1;
 data.continuous_deg=27;SystemControl_Init();
 assert(SystemControl_Apply(&c)==2); /* both origins required */
 c.flags=2;assert(SystemControl_Apply(&c)==0);c.flags=1;assert(SystemControl_Apply(&c)==2);
 c.flags=6;assert(SystemControl_Apply(&c)==0);c.flags=7;assert(SystemControl_Apply(&c)==3);
 ms(1100);assert(SystemControl_Apply(&c)==0);ms(600);assert(idle_wheels());
 c.heading=90;assert(SystemControl_Apply(&c)==0);ms(20);assert(wheels[0]<0&&wheels[1]<0&&wheels[2]>0&&wheels[3]>0);
 data.continuous_deg=117;ms(20);assert(idle_wheels());
 data.continuous_deg=122;ms(20);assert(idle_wheels());ms(510);assert(wheels[0]>0&&wheels[2]<0);
 data.continuous_deg=117;ms(20);assert(idle_wheels());
 c.lift=1;c.slide=2;assert(SystemControl_Apply(&c)==0);ms(510);assert(StepperPulse_Busy());assert(idle_wheels());
 /* repeated frame does not restart an absolute move */
 for(i=0;i<50;i++){assert(SystemControl_Apply(&c)==0);ms(50);assert(idle_wheels());}
 ms(3000);assert(StepperPulse_Position(0)==800);assert(StepperPulse_Position(1)==51);assert(!StepperPulse_Busy());
 c.forward=30;assert(SystemControl_Apply(&c)==0);ms(600);assert(wheels[0]>0&&wheels[1]>0&&wheels[2]>0&&wheels[3]>0);
 c.forward=-30;assert(SystemControl_Apply(&c)==0);ms(20);assert(idle_wheels());ms(510);assert(wheels[0]<0&&wheels[2]<0);
 /* Disabled frame stops, zero must be separate from running. */
 c.flags=0;assert(SystemControl_Apply(&c)==0);assert(idle_wheels());
 c.flags=2;assert(SystemControl_Apply(&c)==0);assert(StepperPulse_Position(0)==0&&StepperPulse_Position(1)==51);
 c.flags=0;assert(SystemControl_Apply(&c)==0);c.flags=4;assert(SystemControl_Apply(&c)==0);assert(!StepperPulse_Position(1));
 c.forward=0;c.heading=90;c.lift=100;c.slide=0;c.flags=5;assert(SystemControl_Apply(&c)==0);ms(1000);n=StepperPulse_Position(0);assert(n>0);
 /* Change latest target; current finite move is not restarted. Disable stops immediately. */
 c.flags=0;assert(SystemControl_Apply(&c)==0);ms(200);assert(StepperPulse_Position(0)==n);
 c.lift=0;c.flags=1;assert(SystemControl_Apply(&c)==0);ms(3000);assert(!StepperPulse_Position(0));
 sensor_valid=0;ms(1);assert(idle_wheels()&&!StepperPulse_Busy());assert(SystemControl_Apply(&c)==4);
 sensor_valid=1;c.flags=0;assert(SystemControl_Apply(&c)==0);c.flags=1;assert(SystemControl_Apply(&c)==0);
 SystemControl_CommsFault();assert(SystemControl_Apply(&c)==4);c.flags=0;assert(SystemControl_Apply(&c)==0);
 g_motor_control.fault=1;ms(1);assert(SystemControl_Apply(&c)==4);
 SystemControl_Status(out,9,4,3);assert(out[0]==0xB5&&out[19]==0xB6&&out[4]==4&&out[15]==2&&out[16]==1);
 for(i=1;i<18;i++)sum^=out[i];assert(out[18]==sum);assert(out[12]==234);
 assert(AxisControl_TargetPulses(0,255)==140000&&AxisControl_TargetPulses(1,255)==4450);
 puts("PASS packet framing/encoding/resync, independent origins, repeated targets, axis/chassis sequencing, yaw hold/correction, reverse brake, disable, sensor/comms/motor faults, telemetry checksum");
}
