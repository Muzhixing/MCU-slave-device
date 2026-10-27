#include <stdint.h>
typedef struct {uint32_t fault;} MotorControlStatus;
extern MotorControlStatus g_motor_control;
void MotorControl_Stop(void);
void MotorSpeedPID_SetTargets(const int32_t *v);
