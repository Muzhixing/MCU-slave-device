# 固件发布文件

新增候选：[`candidate-20261122-a75-feedback`](candidate-20261122-a75-feedback/)，A 上电75°、B/C CAN 回零观察。包含 HEX/BIN/ELF、源码提交标识、SHA256 和验证记录。**未烧录，实际驱动协议及接线未确认，不是已验收生产版。** 使用前读 [接线和验收说明](../docs/startup-feedback-20261122.md)。

以下根目录旧 HEX 保留作为历史发布文件，不对应本次候选：

本目录HEX由本工程2026-10-26清理后全量Rebuild生成。校验见SHA256SUMS，构建日志见../docs/validation/build.log。
目标STM32F407VET6，HSE8MHz，Keil ARMCC5。未在此次整理中烧录。
重新修改源码后应重新构建、复测并更新本目录与校验；不能把旧发布HEX当作最新源码结果。
