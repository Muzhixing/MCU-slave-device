#include "UpperPacket.h"
#include <string.h>
void UpperPacket_Init(UpperParser *p){memset(p,0,sizeof(*p));}
static int Decode(const uint8_t *b,UpperCommand *c){
 int16_t x=b[1]<128?b[1]:(int16_t)b[1]-256;
 int16_t y=b[2]<128?b[2]:(int16_t)b[2]-256;
 if(b[0]!=0xB3 || b[9]!=0xB4 || x < -100 || x>100 || y < -100 || y>100 || b[3]>180 || b[4]>180 || (b[8]&0xF8))return 0;
 c->forward=x;c->lateral=y;c->heading=(uint16_t)(2U*b[3]);
 c->servo=b[4];c->lift=b[5]>175?175:b[5];c->slide=b[6]>175?175:b[6];c->flags=b[8];
 /* b[7] pump request deliberately ignored. */
 return 1;
}
int UpperPacket_Feed(UpperParser *p,uint8_t b,uint32_t ms,UpperCommand *c){
 unsigned i,j;
 if(p->length && (uint32_t)(ms-p->last_ms)>30U)p->length=0;
 p->last_ms=ms;
 if(!p->length && b!=0xB3)return 0;
 p->data[p->length++]=b;if(p->length<10)return 0;
 if(Decode(p->data,c)){p->length=0;return 1;}
 /* Sliding resynchronization; header-valued payload is legal. */
 for(i=1;i<10;i++)if(p->data[i]==0xB3)break;
 for(j=i;j<10;j++)p->data[j-i]=p->data[j];
 p->length=(uint8_t)(10-i);return -1;
}
