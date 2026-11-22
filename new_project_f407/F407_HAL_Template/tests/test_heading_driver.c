#include "HWT101.h"
#include <assert.h>
#include <stdio.h>
USART_TypeDef mock_uart;uint32_t tick_ms;
void USART2_IRQHandler(void);
static void byte(uint8_t b,uint32_t flags){mock_uart.SR=flags;mock_uart.DR=b;USART2_IRQHandler();mock_uart.SR=0;}
static void frame(unsigned ms,int bad){uint8_t f[11]={0x55,0x53,0,0,0,0,0,8,0,0,0};tick_ms=ms;
 for(int i=0;i<10;i++)f[10]+=f[i];if(bad)f[10]^=1;
 for(int i=0;i<11;i++)byte(f[i],USART_SR_RXNE);HWT101_Poll();}
int main(void){HWT101_Diagnostics d;HWT101_Data raw;assert(HWT101_Init()==HAL_OK);
 HWT101_GetDiagnostics(&d);assert(d.version==1&&d.angle_age_ms==UINT32_MAX&&d.rx_bytes==0);
 frame(100,0);frame(120,0);HWT101_GetDiagnostics(&d);assert(d.rx_bytes==22&&d.angle_frames==2&&d.flags==7);
 frame(140,1);frame(160,0);HWT101_GetDiagnostics(&d);assert(d.checksum_errors==1&&d.angle_frames==3&&(d.flags&2));
 tick_ms=170;byte(0,USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE|USART_SR_RXNE);HWT101_Poll();
 HWT101_GetDiagnostics(&d);assert(d.uart_errors==1&&d.overrun_errors==1&&d.framing_errors==1&&d.noise_errors==1&&d.parity_errors==1&&d.discontinuities==1&&d.flags==8);
 frame(180,0);frame(200,0);HWT101_GetDiagnostics(&d);assert(d.flags==13&&d.angle_frames==5);
 assert(!HWT101_Recover(d.discontinuities+1,d.angle_frames));assert(!HWT101_Recover(d.discontinuities,d.angle_frames-1));
 assert(HWT101_Recover(d.discontinuities,d.angle_frames));HWT101_GetDiagnostics(&d);assert(d.flags==3&&d.recoveries==1);
 frame(220,0);HWT101_GetDiagnostics(&d);assert(d.flags==7&&d.last_break_reason==HWT101_BREAK_DRIVER);
 tick_ms=230;for(unsigned i=0;i<256;i++)byte(0,USART_SR_RXNE);HWT101_Poll();HWT101_GetDiagnostics(&d);
 assert(d.queue_overflows==1&&d.uart_errors==1&&d.last_break_reason==HWT101_BREAK_OVERFLOW&&HWT101_RxErrors()==2);
 frame(240,0);frame(260,0);tick_ms=800;HWT101_Poll();HWT101_GetDiagnostics(&d);
 assert(d.last_break_reason==HWT101_BREAK_TIMEOUT&&d.angle_age_ms==540&&d.flags==8);
 frame(820,0);frame(840,0);assert(HWT101_Read(&raw)&&raw.rate_valid&&!raw.continuous_valid);
 /* A partial-frame gap is diagnosed separately from a UART error. */
 tick_ms=850;byte(0x55,USART_SR_RXNE);HWT101_Poll();tick_ms=900;byte(0x55,USART_SR_RXNE);HWT101_Poll();
 HWT101_GetDiagnostics(&d);assert(d.frame_gap_errors==1&&d.uart_errors==1&&d.rx_bytes==HWT101_RxBytes());
 puts("PASS actual USART2 driver: ORE/FE/NE/PE classification, queue overflow, bad checksum, partial-frame gap, age, recovery epoch/sequence guards");
 return 0;}
