#ifndef SYSTEM_CONTROL_H
#define SYSTEM_CONTROL_H
#include "UpperPacket.h"
void SystemControl_Init(void);
void SystemControl_Update(void);
uint8_t SystemControl_Apply(const UpperCommand *cmd);
void SystemControl_CommsFault(void);
void SystemControl_Status(uint8_t out[20],uint8_t seq,uint8_t result,uint8_t errors);
enum { HEADING_STARTUP=0, HEADING_READY=1, HEADING_WAIT_STABLE=2,
       HEADING_WAIT_FRESH=3, HEADING_WAIT_ACK=4 };
typedef struct {
 uint32_t version, phase, losses, recoveries, rejected_enables;
 uint32_t stable_samples, stable_elapsed_ms, last_loss_ms, last_recovery_ms;
 uint32_t raw_age_ms, stream_epoch, rearm_required;
} HeadingDiagnostics;
extern volatile HeadingDiagnostics g_heading_diagnostics;
void SystemControl_GetHeadingDiagnostics(HeadingDiagnostics *out);
#endif
