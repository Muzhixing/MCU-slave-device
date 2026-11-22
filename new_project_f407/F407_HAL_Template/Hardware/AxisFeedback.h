#ifndef AXIS_FEEDBACK_H
#define AXIS_FEEDBACK_H
#include <stdint.h>
/* Read-only Emm V5-compatible 0x3B homing flags. No homing trigger is sent. */
enum { AXIS_WAIT_BUS=0, AXIS_WAIT_HOME, AXIS_HOMING, AXIS_SETTLING,
       AXIS_HOME_VERIFIED, AXIS_FEEDBACK_LOST, AXIS_HOME_FAILED, AXIS_HOME_TIMEOUT };
typedef struct {
 uint32_t phase, epoch, samples, rejected, last_rx_ms, home_started_ms;
 uint32_t settle_started_ms, settle_samples, boot_ms;
 uint8_t flags, seen, verified, reserved;
} AxisFeedbackState;
void AxisFeedback_Init(uint32_t now);
void AxisFeedback_Update(uint32_t now);
void AxisFeedback_Receive(unsigned axis,uint8_t flags,uint32_t now);
void AxisFeedback_Invalidate(unsigned axis,uint32_t now);
uint8_t AxisFeedback_Fresh(unsigned axis,uint32_t now);
uint8_t AxisFeedback_Verified(unsigned axis,uint32_t now);
uint32_t AxisFeedback_Epoch(unsigned axis);
uint8_t AxisFeedback_Failed(void);
void AxisFeedback_Get(unsigned axis,AxisFeedbackState *out);
extern volatile AxisFeedbackState g_axis_feedback[2];
#define AXIS_FEEDBACK_STALE_MS 500U
#define AXIS_HOME_TIMEOUT_MS 30000U
#define AXIS_HOME_SETTLE_MS 200U
#endif
