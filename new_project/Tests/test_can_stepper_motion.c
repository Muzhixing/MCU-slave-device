/**
 ******************************************************************************
 * @file    test_can_stepper_motion.c
 * @brief   Host regression test for the original CAN stepper motion data.
 * @pin_resources No physical pins; host-side CAN command verification only.
 * @function Verifies millimetre conversion and the emitted 0xFD command bytes.
 * @purpose Prevents addresses, directions, RPM, acceleration, pulse counts,
 *          absolute-position mode and synchronization flags from changing.
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "tower.h"

CAN_HandleTypeDef hcan;

static uint8_t captured_command[16];
static uint8_t captured_length;

void can_SendCmd(uint8_t *command, uint8_t length)
{
    memcpy(captured_command, command, length);
    captured_length = length;
}

static int expect_command(const uint8_t *expected, uint8_t length,
                          const char *case_name)
{
    if ((captured_length != length) ||
        (memcmp(captured_command, expected, length) != 0))
    {
        fprintf(stderr, "FAIL: %s CAN command changed\n", case_name);
        return 0;
    }
    return 1;
}

int main(void)
{
    static const uint8_t rail_50_mm[] =
    {
        0x01U, 0xFDU, 0x01U, 0x00U, 0x64U, 0x05U,
        0x00U, 0x00U, 0x9CU, 0x40U, 0x01U, 0x00U, 0x6BU
    };
    static const uint8_t gear_50_mm[] =
    {
        0x02U, 0xFDU, 0x00U, 0x00U, 0x0AU, 0x05U,
        0x00U, 0x00U, 0x04U, 0xF9U, 0x01U, 0x00U, 0x6BU
    };
    static const uint8_t rail_clamped_180_mm[] =
    {
        0x01U, 0xFDU, 0x01U, 0x00U, 0x64U, 0x05U,
        0x00U, 0x02U, 0x32U, 0x80U, 0x01U, 0x00U, 0x6BU
    };

    Rail_StepMotor_ControlByMM(50U, 1U, 1U, 100U, 5U, true, false);
    if (!expect_command(rail_50_mm, sizeof(rail_50_mm), "rail 50 mm"))
    {
        return 1;
    }

    Gear_StepMotor_ControlByMM(50U, 2U, 0U, 10U, 5U, true, false);
    if (!expect_command(gear_50_mm, sizeof(gear_50_mm), "gear 50 mm"))
    {
        return 1;
    }

    Rail_StepMotor_ControlByMM(200U, 1U, 1U, 100U, 5U, true, false);
    if (!expect_command(rail_clamped_180_mm,
                        sizeof(rail_clamped_180_mm), "rail clamp"))
    {
        return 1;
    }

    puts("PASS: CAN stepper commands match the original project data");
    return 0;
}
