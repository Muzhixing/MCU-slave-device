# 正式状态机整合 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将已验证的四轮、HWT101 I2C、横向舵机和 CAN 双步进电机整合为唯一正式状态机入口。

**Architecture:** `main.c` 只负责 HAL、时钟和状态机循环；`State_Machine.c` 负责初始化、安全状态和周期调度；`UpperComputer.c` 只负责固定 10 字节包解析及命令执行。底层驱动保留现有已验证数据和接口。

**Tech Stack:** STM32F103C8T6、STM32 HAL、Keil ARMCC 5、主机 GCC 行为测试。

**Spec:** `docs/superpowers/specs/2026-09-09-original-state-machine-integration-design.md`

## Global Constraints

- 固定包必须保持 `B3 VX VY YAW SERVO LIFT CROSSBAR SW1 SW2 B4`。
- PID 参数、麦轮公式、步进地址和运动参数不变。
- D 电机 PWM 固定使用 PA9/TIM1_CH2；CAN 固定使用 PA11/PA12。
- 未验证的超声波、水泵和牛奶监测不得产生动作。
- 不提交、不推送，不保留 Keil 构建产物和最终测试目录。

---

### Task 1: 固定协议与命令执行

**Files:**
- Modify: `new_project/Hardware/UpperComputer.c`
- Modify: `new_project/Hardware/UpperComputer.h`
- Test: `new_project/Tests/test_upper_computer.c`（整合完成后删除）

**Interfaces:**
- Consumes: `UART_StartReceiveIT(handler)`、`Servo_SetAngle_2(uint8_t)`、`Rail_StepMotor_ControlByMM(...)`、`Gear_StepMotor_ControlByMM(...)`
- Produces: `UpperComputer_Init()`、`UpperComputer_ProcessPacket()`、`UART_GetLatestPacket()`及正式命令值

- [ ] 增加失败测试：ASCII摇杆包不再接受；合法10字节包更新VX/VY/YAW/舵机，并仅在步进字段变化时下发。
- [ ] 运行主机测试，确认因正式接口和行为尚未实现而失败。
- [ ] 移除ASCII摇杆解析、周期运动和测试串口文字，补充横向舵机下发。
- [ ] 运行测试，确认固定包解析、非法包保护和字段变化触发全部通过。

### Task 2: 正式状态机

**Files:**
- Create: `new_project/App/State_Machine.c`
- Create: `new_project/App/State_Machine.h`
- Test: `new_project/Tests/test_state_machine.c`（整合完成后删除）

**Interfaces:**
- Consumes: UART、WT101、Motor、Servo、CAN、Emm_V5和UpperComputer公开接口
- Produces: `State_Machine_Init()`、`State_Machine_Update()`、`State_Machine_GetCurrentState()`

- [ ] 增加失败测试：初始化保持底盘停止、钢丝归零等待30秒、首帧与首个有效Yaw前禁止运动、I2C失败停止电机、合法包进入解析状态。
- [ ] 运行主机测试，确认因状态机文件不存在而失败。
- [ ] 实现 INIT/IDLE/UART_PARSE/ERROR，使用10 ms HWT101周期并保持原PID数据链。
- [ ] 运行测试，确认状态转换、初始化顺序和安全停止行为通过。

### Task 3: 唯一正式入口与工程清理

**Files:**
- Modify: `new_project/User/main.c`
- Modify: `new_project/project.uvprojx`
- Delete: `new_project/App/App_Test.c`, `new_project/App/App_Test.h`
- Delete: `new_project/App/buleteethtest.c`, `new_project/App/buleteethtest.h`
- Delete: `new_project/Tests/`

**Interfaces:**
- Consumes: `State_Machine_Init()`和`State_Machine_Update()`
- Produces: 单一正式固件入口

- [ ] 将 `main.c` 改为初始化时钟后只运行状态机。
- [ ] 在Keil工程中加入状态机并移除实验源文件。
- [ ] 在删除测试目录前运行全部主机测试并保存结果。
- [ ] 删除实验代码和测试目录，检查工程中不存在APP_TEST_MODE、MAIN_APP_MODE及ASCII摇杆入口。

### Task 4: 文档与完整构建

**Files:**
- Modify: `README.md`
- Modify: `log/main.md`
- Modify: `log/20260910.md`

**Interfaces:**
- Consumes: 最终代码和引脚表
- Produces: 与实际固件一致的工程说明

- [ ] 更新PA9、PA7、状态机、10字节包和未完成模块说明。
- [ ] 运行 `git diff --check` 并验证 `project.uvprojx` XML。
- [ ] 用Keil执行完整Rebuild，要求0 Error、0 Warning。
- [ ] 检查Git状态，确认没有提交和推送，也没有新增Keil垃圾文件。
