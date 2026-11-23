# 固件发布文件

当前已烧录版本为 [20261123-a73-startup](20261123-a73-startup/README.md)：A上电回到现场确认的73°，保留四轮滤波15，并与RK3588的自动准备流程配套。

上一四轮验证基线为 [20261123-all-encoder-filter](20261123-all-encoder-filter/README.md)，适用于STM32F407VET6、HSE8MHz。该目录提供GNU Arm 10.3.1生成的BIN、HEX、匹配ELF/MAP、源码哈希与构建记录。

四路编码器输入滤波6→15已在重启后核对；悬空8%速度目标前后各120秒均无故障、全程平滑。范围与证据见 [验收记录](../docs/four-wheel-filter-20261123.md)。

本目录根部的 `F407_HAL_Template.hex` 和 `SHA256SUMS` 是2026-10-26的Keil ARMCC5历史发布，构建日志为 `../docs/validation/build.log`，不包含本次滤波修正。保留用于追溯，不作为当前版本下载入口。

重新构建的输出仍是候选映像；工具链或源码改变后，需建立对应的构建与实机验证记录。
