#ifndef BLUETOOTH_H
#define BLUETOOTH_H
#include "main.h"
#define BLUETOOTH_BAUD_RATE 115200U
typedef struct {
  uint32_t valid_packets, invalid_packets, rx_errors, connected;
  int32_t target[4];
} BluetoothStatus;
extern volatile BluetoothStatus g_bluetooth;
HAL_StatusTypeDef Bluetooth_Init(void);
void Bluetooth_Update(void);
#endif
