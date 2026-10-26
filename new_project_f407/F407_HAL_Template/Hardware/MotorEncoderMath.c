#include "MotorEncoder.h"

int32_t MotorEncoder_Delta(uint16_t previous, uint16_t current)
{
  uint32_t delta = (uint16_t)(current - previous);
  return delta < 32768U ? (int32_t)delta : (int32_t)delta - 65536;
}

int32_t MotorEncoder_PulseToSpeed(int32_t pulses, uint32_t elapsed_ms)
{
  if (!elapsed_ms) { return 0; }
  return (int32_t)(((int64_t)pulses * 60000 * MOTOR_ENCODER_SPEED_SCALE) /
    ((int64_t)MOTOR_ENCODER_RESOLUTION * MOTOR_ENCODER_GEAR_RATIO * elapsed_ms));
}
