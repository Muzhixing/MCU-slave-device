#include "AxisFeedback.h"
#include <string.h>
volatile AxisFeedbackState g_axis_feedback[2];
static void Lost(volatile AxisFeedbackState *s,uint32_t phase){
 s->phase=phase;s->verified=0;s->settle_samples=0;
}
void AxisFeedback_Init(uint32_t now){
 unsigned a;memset((void *)g_axis_feedback,0,sizeof(g_axis_feedback));
 for(a=0;a<2;a++)g_axis_feedback[a].boot_ms=now;
}
void AxisFeedback_Invalidate(unsigned a,uint32_t now){
 (void)now;if(a<2){g_axis_feedback[a].rejected++;Lost(&g_axis_feedback[a],AXIS_FEEDBACK_LOST);}
}
uint8_t AxisFeedback_Fresh(unsigned a,uint32_t now){
 return a<2 && g_axis_feedback[a].seen &&
  (uint32_t)(now-g_axis_feedback[a].last_rx_ms)<AXIS_FEEDBACK_STALE_MS;
}
void AxisFeedback_Update(uint32_t now){
 unsigned a;for(a=0;a<2;a++){
  volatile AxisFeedbackState *s=&g_axis_feedback[a];
  if(s->seen && !AxisFeedback_Fresh(a,now))Lost(s,AXIS_FEEDBACK_LOST);
  else if((s->phase==AXIS_HOMING || s->phase==AXIS_SETTLING) &&
          (uint32_t)(now-s->home_started_ms)>=AXIS_HOME_TIMEOUT_MS)Lost(s,AXIS_HOME_TIMEOUT);
 }
}
void AxisFeedback_Receive(unsigned a,uint8_t flags,uint32_t now){
 volatile AxisFeedbackState *s;uint8_t stale;
 if(a>=2)return;
 s=&g_axis_feedback[a];stale=s->seen && !AxisFeedback_Fresh(a,now);
 if(stale)Lost(s,AXIS_FEEDBACK_LOST);
 /* Upper flag bits are reserved in the supported Emm V5 homing response. */
 if(flags&0xF0U){AxisFeedback_Invalidate(a,now);return;}
 s->flags=flags;s->last_rx_ms=now;s->seen=1;s->samples++;
 if((flags&3U)!=3U || (flags&8U)){Lost(s,AXIS_HOME_FAILED);return;}
 if(flags&4U){
  /* Timeout remains latched until a non-busy report, then a new busy edge. */
  if(s->phase==AXIS_HOME_TIMEOUT)return;
  if(s->phase!=AXIS_HOMING && s->phase!=AXIS_SETTLING){
   s->epoch++;s->home_started_ms=now;
  }
  s->phase=AXIS_HOMING;s->verified=0;s->settle_samples=0;
 }else if(s->phase==AXIS_HOMING){
  s->phase=AXIS_SETTLING;s->settle_started_ms=now;s->settle_samples=1;
 }else if(s->phase==AXIS_SETTLING){
  s->settle_samples++;
  if(s->settle_samples>=3 && (uint32_t)(now-s->settle_started_ms)>=AXIS_HOME_SETTLE_MS){
   s->phase=AXIS_HOME_VERIFIED;s->verified=1;
  }
 }else if(s->phase!=AXIS_HOME_VERIFIED){
  /* Idle/ready on first contact is NOT evidence of a completed homing cycle. */
  s->phase=AXIS_WAIT_HOME;s->verified=0;
 }
 AxisFeedback_Update(now);
}
uint8_t AxisFeedback_Verified(unsigned a,uint32_t now){
 return AxisFeedback_Fresh(a,now) && g_axis_feedback[a].verified;
}
uint32_t AxisFeedback_Epoch(unsigned a){return a<2?g_axis_feedback[a].epoch:0;}
uint8_t AxisFeedback_Failed(void){
 unsigned a;for(a=0;a<2;a++)if(g_axis_feedback[a].phase>=AXIS_FEEDBACK_LOST)return 1;
 return 0;
}
void AxisFeedback_Get(unsigned a,AxisFeedbackState *out){if(a<2 && out)*out=g_axis_feedback[a];}
