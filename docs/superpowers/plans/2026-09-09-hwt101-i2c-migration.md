# HWT101 I2C Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the new project's HWT101 USART1 transport with the original 100 kHz I2C1 register protocol on PB6/PB7 and provide a standalone Bluetooth-readable hardware test.

**Architecture:** `I2C.c` owns I2C1 initialization, while `WT101.c` owns device readiness and the two-byte Yaw register read. Application code consumes a status-returning `WT101_ReadYaw()` interface so an I2C fault is never converted into a valid zero-degree sample.

**Tech Stack:** STM32F103C8T6, STM32 HAL, I2C1, USART2 debug output, Keil MDK-ARM, host C regression tests.

**Spec:** `docs/superpowers/specs/2026-09-09-original-state-machine-integration-design.md`

## Global Constraints

- Use PB6 as I2C1 SCL and PB7 as I2C1 SDA at 100 kHz.
- Preserve HWT101 7-bit address `0x50`, Yaw register `0x3F`, little-endian two-byte data and `raw / 32768 * 180` scaling.
- Interpret raw Yaw as signed `int16_t`.
- Use finite HAL timeouts; never use `HAL_MAX_DELAY` and never return a fabricated `0°` on failure.
- Do not modify motor, CAN, stepper, servo or state-machine behavior in this stage.
- Do not commit or push implementation changes unless the user separately requests it.

---

### Task 1: Define and test the I2C HWT101 interface

**Files:**
- Modify: `new_project/Tests/stubs/stm32f1xx_hal.h`
- Replace: `new_project/Tests/test_wt101_config.c`
- Modify: `new_project/Hardware/WT101.h`

**Interfaces:**
- Consumes: `extern I2C_HandleTypeDef hi2c1`
- Produces: `void WT101_Init(void)`, `HAL_StatusTypeDef WT101_IsReady(uint32_t timeout_ms)`, `HAL_StatusTypeDef WT101_ReadYaw(float *yaw_angle, uint32_t timeout_ms)`

- [ ] **Step 1: Write a failing host test**

The test stubs `HAL_I2C_IsDeviceReady()` and `HAL_I2C_Mem_Read()`, verifies device address `0x50 << 1`, register `0x3F`, two-byte little-endian signed conversion, caller timeout propagation, null-pointer rejection and HAL error propagation.

- [ ] **Step 2: Compile and run the test to verify failure**

Run the available host C compiler against `test_wt101_config.c` and `Hardware/WT101.c`. Expected result before implementation: compile failure because `WT101_Init`, `WT101_IsReady` and the I2C form of `WT101_ReadYaw` do not exist.

- [ ] **Step 3: Replace the public UART API**

Define:

```c
#define WT101_I2C_ADDRESS       0x50U
#define WT101_YAW_REGISTER      0x3FU
#define WT101_I2C_DEFAULT_TIMEOUT_MS 10U

void WT101_Init(void);
HAL_StatusTypeDef WT101_IsReady(uint32_t timeout_ms);
HAL_StatusTypeDef WT101_ReadYaw(float *yaw_angle, uint32_t timeout_ms);
```

Remove USART1 handles, frame sizes, UART frame parsing and UART callback declarations from `WT101.h`.

- [ ] **Step 4: Re-run the focused test**

Expected result: interface test compiles; behavior tests still fail until Task 2 provides the implementation.

### Task 2: Implement finite-timeout I2C Yaw reads

**Files:**
- Modify: `new_project/Hardware/I2C.c`
- Modify: `new_project/Hardware/I2C.h`
- Replace: `new_project/Hardware/WT101.c`
- Test: `new_project/Tests/test_wt101_config.c`

**Interfaces:**
- Consumes: `MX_I2C1_Init()`, `hi2c1`, `HAL_I2C_IsDeviceReady`, `HAL_I2C_Mem_Read`
- Produces: the three public `WT101_*` functions from Task 1

- [ ] **Step 1: Make I2C initialization production-oriented**

Keep 100 kHz, 7-bit addressing and PB6/PB7 alternate-function open-drain. Replace test-oriented file comments with the final HWT101 bus responsibility. Preserve the existing HAL initialization values.

- [ ] **Step 2: Implement HWT101 initialization and readiness**

```c
void WT101_Init(void)
{
    MX_I2C1_Init();
}

HAL_StatusTypeDef WT101_IsReady(uint32_t timeout_ms)
{
    return HAL_I2C_IsDeviceReady(&hi2c1,
                                 WT101_I2C_ADDRESS << 1U,
                                 2U,
                                 timeout_ms);
}
```

- [ ] **Step 3: Implement the Yaw register read**

```c
HAL_StatusTypeDef WT101_ReadYaw(float *yaw_angle, uint32_t timeout_ms)
{
    uint8_t data[2];
    int16_t raw;
    HAL_StatusTypeDef status;

    if (yaw_angle == NULL) return HAL_ERROR;
    status = HAL_I2C_Mem_Read(&hi2c1,
                              WT101_I2C_ADDRESS << 1U,
                              WT101_YAW_REGISTER,
                              I2C_MEMADD_SIZE_8BIT,
                              data, 2U, timeout_ms);
    if (status != HAL_OK) return status;
    raw = (int16_t)(((uint16_t)data[1] << 8U) | data[0]);
    *yaw_angle = (float)raw / 32768.0f * 180.0f;
    return HAL_OK;
}
```

- [ ] **Step 4: Run focused host tests**

Expected: positive and negative Yaw conversion passes, address/register/length are exact, and HAL errors propagate unchanged.

### Task 3: Convert the standalone HWT101 hardware test

**Files:**
- Modify: `new_project/App/App_Test.c`
- Modify: `new_project/App/App_Test.h`
- Modify: `new_project/App/App_Test.h` selection to `APP_TEST_WT101`
- Modify: `new_project/App/buleteethtest.c`
- Modify: `new_project/Hardware/UART.c`
- Modify: `new_project/User/stm32f1xx_it.c`
- Modify: `new_project/User/stm32f1xx_it.h`
- Modify: `new_project/User/main.c` comments only where required

**Interfaces:**
- Consumes: `WT101_Init`, `WT101_IsReady`, `WT101_ReadYaw`, `UART_SendString`
- Produces: an isolated test that sends I2C status and Yaw through HC-08 without initializing motors

- [ ] **Step 1: Change the selected temporary hardware mode**

Select `APP_TEST_WT101` so the firmware does not start the chassis during I2C validation.

- [ ] **Step 2: Replace raw UART forwarding with I2C telemetry**

Initialization prints one readiness result. The run step reads Yaw every 100 ms and sends one of:

```text
HWT101 I2C READY\r\n
[HWT,OK,<yaw_x100>]\r\n
[HWT,ERR,<hal_status>]\r\n
```

`yaw_x100` preserves two decimal places without requiring floating-point `printf` support.

- [ ] **Step 3: Remove HWT101 USART1 runtime dependencies**

Remove the USART1 branch from `HAL_UART_RxCpltCallback`, remove `USART1_IRQHandler`, and update remaining temporary Bluetooth code to call the synchronous I2C interface so every compiled source uses the new public API. Do not alter Bluetooth joystick or motor behavior beyond the required HWT101 API substitution.

- [ ] **Step 4: Run host tests for the isolated app path**

Expected: HWT101 test initializes I2C and USART2 only, never calls `Motor_Init`, and emits the documented status packet.

### Task 4: Verify the embedded project

**Files:**
- Modify if needed: `new_project/project.uvprojx`
- Verify: `new_project/User/stm32f1xx_hal_conf.h`

**Interfaces:**
- Consumes: all code from Tasks 1–3
- Produces: a loadable HWT101-I2C-only validation image

- [ ] **Step 1: Verify project membership**

Confirm `Hardware/I2C.c`, `Hardware/WT101.c`, `App/App_Test.c`, USART2 HAL and I2C HAL sources are included. Confirm no source references the removed HWT101 UART API.

- [ ] **Step 2: Run text and diff checks**

Run searches for `WT101_UART`, `huart1`, `USART1_IRQHandler`, `HAL_MAX_DELAY`, address `0x50`, register `0x3F`, and PB6/PB7 modes. Expected: no HWT101 UART dependency and no infinite I2C timeout.

- [ ] **Step 3: Build with Keil**

Build `new_project/project.uvprojx`. Expected: zero errors. Warnings must be inspected and any warning introduced by this migration fixed.

- [ ] **Step 4: Report physical validation procedure**

Connect PB6/SCL and PB7/SDA with approximately 4.7 kΩ pull-ups to 3.3 V, share ground, power HWT101 according to its board specification, and observe `[HWT,OK,...]` through HC-08. Do not enable the chassis in this stage.
