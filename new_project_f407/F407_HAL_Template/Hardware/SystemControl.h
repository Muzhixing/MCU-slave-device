#ifndef SYSTEM_CONTROL_H
#define SYSTEM_CONTROL_H
#include "UpperPacket.h"
void SystemControl_Init(void);
void SystemControl_Update(void);
uint8_t SystemControl_Apply(const UpperCommand *cmd);
void SystemControl_CommsFault(void);
void SystemControl_Status(uint8_t out[20],uint8_t seq,uint8_t result,uint8_t errors);
#endif
