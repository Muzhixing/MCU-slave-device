# 2026-11-23 四轮滤波发布

本目录保存已通过RK3588/ST-Link烧录并验证的完整GNU映像。源码基线为 `785123b41001efa6e673d0bca2a0376d9a7788ff`，其源码树与仓库合并提交 `e58f9dc` 一致；运行逻辑改动为 `Hardware/MotorEncoder.c` 中四路 IC1F/IC2F 从6改为15。

|文件|用途|
|---|---|
|f407_all_encoder_filter.hex|含Flash地址的烧录映像|
|f407_all_encoder_filter.bin|20136字节，装载地址0x08000000|
|f407_all_encoder_filter.elf / .map|与本次映像匹配的调试符号、变量地址与链接布局|
|build_manifest.json|原始构建命令、93个源码输入哈希、产物哈希与工具链版本|
|startup.S / stm32f407ve.ld|本次生成的启动汇编与链接脚本|
|build.log / symbols.txt|原始编译诊断与符号表|
|SHA256SUMS|上述构建产物的SHA-256校验|

BIN SHA-256：`c55dd404078189c1cad35c5ef671315577f26c2d80e8330970c08febff9d4e00`。暂停MCU后的Flash读回与BIN逐字节相同，HEX内容也与BIN一致。

原始构建清单在构建时生成，因此其中 `hardware_validated: false` 表示当时尚未烧录；后续实机验收以 [四轮验收记录](../../docs/four-wheel-filter-20261123.md) 和其链接的验收数据为准。构建清单保留原值用于追溯。

## 重新构建

在工程根目录 `F407_HAL_Template` 下，指定包含 `arm-none-eabi-gcc` 等程序的工具链目录：

```sh
python3 tools/build.py --source . --out /tmp/f407-filter-build --toolchain /path/to/toolchain/bin
python3 tests/run.py
```

本次实际版本为 `arm-none-eabi-gcc (15:10.3-2021.07-4) 10.3.1 20210621 (release)`，在RK3588上构建。GNU编译使用Cortex-M4硬浮点、`-Os`、newlib-nano、HSE8MHz；Flash512KiB、普通SRAM128KiB，预留4KiB栈和2KiB堆。构建工具保留本次源基线标识，新构建不会自动获得已验证状态。

构建日志中有5条既有警告：HAL的3处未使用参数、Servo的2处符号比较。修改的编码器模块没有新增警告。此前Keil构建使用不同启动文件、链接布局和运行库；本发布是完整GNU映像，不能当作对旧HEX的局部字节替换。

本次已验证范围是重启、原协议、零点登记、四轮悬空正反向及禁用停车；没有重新验证ABC运动。负载、麦克纳姆横移/转向、高速和断线自动停车见 [后续验证](../../docs/four-wheel-followups.md)。
