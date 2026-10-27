/* Ported from MCU-slave-device PIDtest c433766c; incremental speed PID. */
#include "MotorSpeedPID.h"

#include "motor.h"
#include <stddef.h>

typedef struct
{
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t target_speed;
    int32_t current_speed;
    int32_t error;
    int32_t last_error;
    int32_t previous_error;
    int32_t output;
    int32_t gain_divisor;
    int32_t feedforward_pwm;
} MotorSpeedPID_Controller_t;

static MotorSpeedPID_Controller_t motor_pid[MOTOR_SPEED_PID_MOTOR_COUNT];
static int32_t motor_target_speed[MOTOR_SPEED_PID_MOTOR_COUNT];
static int32_t motor_measured_speed[MOTOR_SPEED_PID_MOTOR_COUNT];
static uint32_t motor_pid_last_tick;
static int32_t ClampSpeed(int32_t x) { return x > 10000 ? 10000 : (x < -10000 ? -10000 : x); }

static int32_t MotorSpeedPID_ClampMagnitude(int64_t value)
{
    if (value > MOTOR_SPEED_PID_OUTPUT_MAX)
    {
        return MOTOR_SPEED_PID_OUTPUT_MAX;
    }
    if (value < 0)
    {
        return 0;
    }
    return value;
}

static void MotorSpeedPID_ResetController(MotorSpeedPID_Controller_t *controller)
{
    controller->target_speed = 0;
    controller->current_speed = 0;
    controller->error = 0;
    controller->last_error = 0;
    controller->previous_error = 0;
    controller->output = 0;
}

static int32_t MotorSpeedPID_Compute(MotorSpeedPID_Controller_t *controller,
                                     int32_t target_speed,
                                     int32_t current_speed)
{
    int64_t delta_output;
    int32_t direction;
    int32_t target_magnitude;
    int32_t aligned_current_speed;
    uint8_t starting_or_reversing;

    if (target_speed == 0)
    {
        MotorSpeedPID_ResetController(controller);
        return 0;
    }

    starting_or_reversing =
        (uint8_t)((controller->target_speed == 0) ||
                  ((controller->target_speed > 0) && (target_speed < 0)) ||
                  ((controller->target_speed < 0) && (target_speed > 0)));
    if (starting_or_reversing != 0U)
    {
        MotorSpeedPID_ResetController(controller);
        controller->output = controller->feedforward_pwm;
    }

    direction = (target_speed < 0) ? -1 : 1;
    target_magnitude = (target_speed < 0) ? -target_speed : target_speed;
    aligned_current_speed = current_speed * direction;
    controller->target_speed = target_speed;
    controller->current_speed = current_speed;
    controller->error = target_magnitude - aligned_current_speed;
    delta_output = ((int64_t)controller->kp *
                    (controller->error - controller->last_error) +
                    (int64_t)controller->ki * controller->error +
                    (int64_t)controller->kd *
                    (controller->error - (2 * controller->last_error) +
                     controller->previous_error)) /
                   controller->gain_divisor;
    controller->output =
        MotorSpeedPID_ClampMagnitude(controller->output + delta_output);
    controller->previous_error = controller->last_error;
    controller->last_error = controller->error;
    return controller->output * direction;
}

void MotorSpeedPID_Init(void)
{
    uint32_t index;

    for (index = 0U; index < MOTOR_SPEED_PID_MOTOR_COUNT; index++)
    {
        motor_pid[index].kp = MOTOR_SPEED_PID_DEFAULT_KP;
        motor_pid[index].ki = MOTOR_SPEED_PID_DEFAULT_KI;
        motor_pid[index].kd = MOTOR_SPEED_PID_DEFAULT_KD;
        motor_pid[index].gain_divisor = 1;
        motor_pid[index].feedforward_pwm = 0;
        MotorSpeedPID_ResetController(&motor_pid[index]);
        motor_target_speed[index] = 0;
        motor_measured_speed[index] = 0;
    }
    motor_pid_last_tick = HAL_GetTick();
}

void MotorSpeedPID_SetParameters(uint8_t motor_index,
                                 int32_t kp,
                                 int32_t ki,
                                 int32_t kd,
                                 int32_t gain_divisor,
                                 int32_t feedforward_pwm)
{
    MotorSpeedPID_Controller_t *controller;

    if ((motor_index >= MOTOR_SPEED_PID_MOTOR_COUNT) ||
        (gain_divisor <= 0 || kp < 0 || ki < 0 || kd < 0 || kp > 10000 || ki > 10000 || kd > 10000))
    {
        return;
    }

    controller = &motor_pid[motor_index];
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->gain_divisor = gain_divisor;
    controller->feedforward_pwm =
        MotorSpeedPID_ClampMagnitude(feedforward_pwm);
    MotorSpeedPID_ResetController(controller);
}

void MotorSpeedPID_SetTargets(
    const int32_t targets[MOTOR_SPEED_PID_MOTOR_COUNT])
{
    uint32_t index;

    if (targets == NULL)
    {
        return;
    }
    for (index = 0U; index < MOTOR_SPEED_PID_MOTOR_COUNT; index++)
    {
        motor_target_speed[index] = ClampSpeed(targets[index]);
    }
}

void MotorSpeedPID_SetMeasuredSpeeds(
    const int32_t measured_speeds[MOTOR_SPEED_PID_MOTOR_COUNT])
{
    uint32_t index;

    if (measured_speeds == NULL)
    {
        return;
    }
    for (index = 0U; index < MOTOR_SPEED_PID_MOTOR_COUNT; index++)
    {
        motor_measured_speed[index] = ClampSpeed(measured_speeds[index]);
    }
}

uint8_t MotorSpeedPID_RunStep(void)
{
    int32_t output[MOTOR_SPEED_PID_MOTOR_COUNT];
    uint32_t now = HAL_GetTick();
    uint32_t index;

    if ((now - motor_pid_last_tick) < MOTOR_SPEED_PID_PERIOD_MS)
    {
        return 0U;
    }
    motor_pid_last_tick = now;

    for (index = 0U; index < MOTOR_SPEED_PID_MOTOR_COUNT; index++)
    {
        output[index] = MotorSpeedPID_Compute(&motor_pid[index],
                                              motor_target_speed[index],
                                              motor_measured_speed[index]);
    }

    for (index = 0; index < MOTOR_SPEED_PID_MOTOR_COUNT; ++index) {
        HAL_StatusTypeDef result = Motor_Set((MotorId)index, output[index]);
        if (result == HAL_BUSY) {
            /* Preserve the driver's 500 ms stop-before-reverse interlock. */
            (void)Motor_Set((MotorId)index, 0);
            MotorSpeedPID_ResetController(&motor_pid[index]);
        } else if (result != HAL_OK) {
            MotorSpeedPID_Stop();
            return 2U;
        }
    }
    return 1U;
}

void Motor_A_SetTargetSpeed(int32_t speed)
{
    motor_target_speed[0] = ClampSpeed(speed);
}

void Motor_B_SetTargetSpeed(int32_t speed)
{
    motor_target_speed[1] = ClampSpeed(speed);
}

void Motor_C_SetTargetSpeed(int32_t speed)
{
    motor_target_speed[2] = ClampSpeed(speed);
}

void Motor_D_SetTargetSpeed(int32_t speed)
{
    motor_target_speed[3] = ClampSpeed(speed);
}

int32_t MotorSpeedPID_GetTarget(uint8_t motor_index)
{
    return (motor_index < MOTOR_SPEED_PID_MOTOR_COUNT) ?
           motor_target_speed[motor_index] : 0;
}

int32_t MotorSpeedPID_GetMeasuredSpeed(uint8_t motor_index)
{
    return (motor_index < MOTOR_SPEED_PID_MOTOR_COUNT) ?
           motor_measured_speed[motor_index] : 0;
}

int32_t MotorSpeedPID_GetOutput(uint8_t motor_index)
{
    if (motor_index >= MOTOR_SPEED_PID_MOTOR_COUNT)
    {
        return 0;
    }
    return (motor_pid[motor_index].target_speed < 0) ?
           -motor_pid[motor_index].output : motor_pid[motor_index].output;
}

void MotorSpeedPID_Stop(void)
{
    uint32_t i;
    Motor_StopAll();
    for(i=0; i<4; ++i) { motor_target_speed[i]=0; MotorSpeedPID_ResetController(&motor_pid[i]); }
}
