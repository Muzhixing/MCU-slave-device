# STM32F407 小车整合工程

当前唯一维护工程：**[F407_HAL_Template](new_project_f407/F407_HAL_Template/README.md)**。

STM32F407VET6 · Keil 5 / ARMCC 5 或 GNU Arm 10.3.1 · STM32 HAL · ST-Link/SWD。
整合四轮速度PID、HWT101航向闭环、180°舵机、超声波、升降与横杆脉冲步进轴。水泵不参与运行，仅保留关断初始化。

当前已烧录版本为 **A73 上电回正**：现场重新校准机械正中为73°，复位后仍居中；RK3588配套自动完成8秒等待及B/C登记。见 [启动修复与验收](new_project_f407/F407_HAL_Template/docs/startup-zero-20261123.md)。

四轮稳定性基线为 **2026-11-23 四轮编码器滤波修正**：四路输入滤波从6调整为15，悬空8%目标前进、后退各120秒均无故障，用户确认全程平滑。源码、已烧录BIN/HEX、匹配的ELF/MAP及验证记录一并归档；详见 [发布文件](new_project_f407/F407_HAL_Template/firmware/README.md) 与 [四轮验收](new_project_f407/F407_HAL_Template/docs/four-wheel-filter-20261123.md)。

- [接线及资源分配](new_project_f407/F407_HAL_Template/docs/wiring.md)
- [串口二进制协议与测试包](new_project_f407/F407_HAL_Template/docs/protocol.md)
- [状态机](new_project_f407/F407_HAL_Template/docs/state-machine.md)
- [逐项测试记录](new_project_f407/F407_HAL_Template/docs/test-history.md)
- [复测步骤](new_project_f407/F407_HAL_Template/docs/test-guide.md)
- [原理图与硬件资料](new_project_f407/F407_HAL_Template/docs/hardware/README.md)

旧F103工程、临时资料、重复上游工程和阶段测试入口已移出工作目录；原有Git历史保留。清理前完整文件归档保存在本机工程目录外，详情见 [整理记录](new_project_f407/F407_HAL_Template/docs/cleanup.md)。
