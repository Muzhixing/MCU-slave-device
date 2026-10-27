#ifndef UPPER_PACKET_H
#define UPPER_PACKET_H
#include <stdint.h>
typedef struct {int16_t forward,lateral;uint16_t heading;uint8_t servo,lift,slide,flags;} UpperCommand;
typedef struct {uint8_t data[10],length;uint32_t last_ms;} UpperParser;
void UpperPacket_Init(UpperParser *p);
/* 0=incomplete, 1=valid, -1=invalid frame. */
int UpperPacket_Feed(UpperParser *p,uint8_t byte,uint32_t ms,UpperCommand *cmd);
#endif
