# main 分支说明

## 当前定位

`main` 以提交 `d0c4b19` 的四路开环蓝牙地盘程序为主体，并合并 `CANtest` 的双步进电机代码。四路编码器 PID 实验不作为主分支默认功能，独立保存在 `PIDtest` 分支。

当前工程保留两种互斥测试入口：

1. 四路开环麦克纳姆轮地盘，默认运行。
2. CAN 控制钢丝和横杆两台闭环步进电机，按需切换。

这两套功能已经分别完成实验，但按当前引脚分配不能同时运行。

## 默认四路开环地盘

入口在 `new_project/User/main.c`，默认配置为：

```c
#define MAIN_MODE_OPEN_LOOP_CHASSIS  1U
#define MAIN_MODE_CAN_STEPPER        2U

#ifndef MAIN_APP_MODE
#define MAIN_APP_MODE MAIN_MODE_OPEN_LOOP_CHASSIS
#endif
```

`APP_TEST_MODE` 继续默认选择 `APP_TEST_BLUETOOTH`。HC-08 通过 PA2/PA3 的 USART2 接收 `[j,LX,LY,RX,RY]`，程序直接完成麦克纳姆轮运动分配和 PWM 输出，不读取编码器，也不执行电机速度 PID 或 WT101 航向 PID。

地盘接线沿用开环实验：

| 电机 | PWM | 方向引脚 |
| --- | --- | --- |
| A / LF | PA0 | PB5、PB12 |
| B / LB | PA1 | PB13、PB14 |
| C / RF | PA8 | PB15、PA4 |
| D / RB | PA11 | PB3、PB4 |

## CAN 步进模式

需要测试步进电机时，把 `MAIN_APP_MODE` 改为 `MAIN_MODE_CAN_STEPPER`：

- PA12/CAN1_TX 接外接 TJA 的 TXD。
- PA11/CAN1_RX 接外接 TJA 的 RXD。
- TJA 的 CANH/CANL 接两台驱动器的 CHAN/CHAL，并与系统共地。
- CAN1 使用 Normal 模式、500 kbit/s、扩展帧和固定校验字节 `0x6B`。
- 钢丝电机地址为 1，横杆电机地址为 2。
- 左摇杆 Y 控制钢丝，右摇杆 Y 控制横杆。
- 钢丝在开放蓝牙控制前先执行无感归零流程。

详细接线、协议和驱动器参数见 `log/20260908.md`，独立测试版本保存在 `CANtest` 分支。

## 引脚冲突

| 引脚 | 开环地盘 | CAN 步进模式 |
| --- | --- | --- |
| PA11 | 电机 D / RB PWM | CAN1_RX |
| PA12 | 当前开环地盘不使用 | CAN1_TX |

因此程序通过 `MAIN_APP_MODE` 只运行一种模式。最终整机若要同时运行地盘与 CAN 步进电机，需要重新分配 CAN 或电机 PWM 引脚，并完成新的整机验证。

## 分支关系

- `main`：四路开环地盘为默认入口，并包含可切换的 CAN 步进测试。
- `PIDtest`：四路编码器速度 PID 地盘实验。
- `CANtest`：CAN 双步进电机与蓝牙控制的独立实验。
