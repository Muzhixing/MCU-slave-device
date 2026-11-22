#ifndef HWT101_H
#define HWT101_H
#include "main.h"
#include "HWT101_Parser.h"
/* Read-only USART2 RX: PA3 <- sensor TX, 115200 8N1. PA2 untouched. */
HAL_StatusTypeDef HWT101_Init(void);
void HWT101_Poll(void);
uint8_t HWT101_Read(HWT101_Data *data);
uint8_t HWT101_Zero(void);
uint8_t HWT101_Recover(uint32_t expected_epoch, uint32_t expected_sequence);
uint32_t HWT101_RxBytes(void);
uint32_t HWT101_RxErrors(void);
/* SWD/debugger-readable diagnostics. No new UART frames or field reuse. */
typedef struct {
 uint32_t version, rx_bytes, uart_errors, overrun_errors, framing_errors;
 uint32_t noise_errors, parity_errors, queue_overflows, last_rx_ms, last_error_ms;
 uint32_t valid_frames, angle_frames, checksum_errors, frame_gap_errors, discarded_bytes;
 uint32_t discontinuities, recoveries, last_angle_ms, angle_age_ms;
 uint32_t flags, last_break_reason;
} HWT101_Diagnostics;
/* flags: bit0 raw-valid, bit1 continuous-valid, bit2 rate-valid, bit3 broken. */
extern volatile HWT101_Diagnostics g_hwt101_diagnostics;
void HWT101_GetDiagnostics(HWT101_Diagnostics *out);
#endif
