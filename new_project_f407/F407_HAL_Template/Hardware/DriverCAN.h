#ifndef DRIVER_CAN_H
#define DRIVER_CAN_H
#include "main.h"
/* CAN1 PA11 RX / PA12 TX -> external transceiver logic pins.
 * Never connect these logic pins directly to CANH/CANL. */
HAL_StatusTypeDef DriverCAN_Init(void);
void DriverCAN_Poll(void);
/* Parser also used by host tests. Extended data frame, packet number zero. */
uint8_t DriverCAN_Decode(uint32_t id,uint8_t extended,uint8_t remote,
                        const uint8_t *data,uint8_t length,unsigned *axis,uint8_t *flags);
typedef struct {uint32_t version,tx,rx,rejected,tx_failures,bus_faults,hal_error;} DriverCANDiagnostics;
extern volatile DriverCANDiagnostics g_driver_can;
#define DRIVER_B_ADDRESS 1U
#define DRIVER_C_ADDRESS 2U
#endif
