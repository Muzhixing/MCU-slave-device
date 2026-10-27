# F407 HAL 整合固件

## 打开与编译

打开 `MDK-ARM/F407_HAL_Template.uvprojx`，目标 `F407_HAL_Template`。
配置为 STM32F407VETx、ARMCC V5.06 update 5 build 528、Keil.STM32F4xx_DFP 2.17.1。不要将编译器自动切换为ARMCLANG后直接使用未经验证的结果。器件包版本升级需要另行验证；历史安装过3.1.1不代表本工程当前依赖已更改。

HSE=8MHz，PLL M=8/N=336/P=2/Q=7，SYSCLK=168MHz，APB1=42MHz，APB2=84MHz。Flash为0x08000000起512KB，普通SRAM为0x20000000起128KB；未使用CCM。

Keil Rebuild后生成 `MDK-ARM/Objects/F407_HAL_Template.hex`。ST-Link选择SWD，并使用覆盖0x08000000的STM32F4xx 512KB Flash算法。出现“No Algorithm found”应检查Flash Download算法，不应修改运行时PLL代码解决。

## 目录

|目录|职责|
|---|---|
|User|启动入口、时钟、中断、HAL配置|
|Hardware|协议、状态机及实际外设驱动.c/.h|
|Drivers|HAL/CMSIS依赖及原许可证|
|MDK-ARM|Keil工程、ST-Link配置和启动汇编|
|tests|主机回归测试及硬件桩|
|docs|接线、协议、历史测试及原始硬件资料|
|firmware|本次构建发布的HEX、校验及构建说明|

`python tests/run.py` 运行GCC主机测试（需要Python3和PATH中的gcc），测试直接复制当前生产模块到临时目录编译，测试结束自动清理临时输出。

## 使用

1. 在实际机械零位上电：lift最顶部、slide约定起点。固件不会自动寻找零点。
2. 保持车体静止，HWT101稳定约1秒建立上电航向。
3. 串口115200、8N1，按 [协议](docs/protocol.md) 发送HEX字节；先确认零点，再使能。
4. 默认底盘和步进轴不运动，舵机默认90°，水泵保持关闭。运行命令持续有效，没有500ms无报文停车逻辑。

完整行为见 [状态机](docs/state-machine.md)，现场复测见 [测试步骤](docs/test-guide.md)。步进位置来自已发脉冲累计，并非实测位置；掉电、丢步或手动移动后必须恢复机械基准再确认零点。

## 来源

最初HAL模板参考 GDUT-ELC/STM32-source-code 的01-LED；电机控制从本仓库F103阶段代码迁移。现仅保留F407生产入口及有用原始资料；第三方HAL/CMSIS许可证保留在Drivers中，本次整理不改变其许可。
