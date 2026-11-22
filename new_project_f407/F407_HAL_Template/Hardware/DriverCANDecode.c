#include "DriverCAN.h"
uint8_t DriverCAN_Decode(uint32_t id,uint8_t ext,uint8_t remote,const uint8_t *d,
                        uint8_t n,unsigned *axis,uint8_t *flags){
 if(!ext || remote || !d || !axis || !flags || n!=3 || d[0]!=0x3BU || d[2]!=0x6BU)return 0;
 if(id==(DRIVER_B_ADDRESS<<8))*axis=0;
 else if(id==(DRIVER_C_ADDRESS<<8))*axis=1;
 else return 0;
 if(d[1]&0xF0U)return 0;
 *flags=d[1];return 1;
}
