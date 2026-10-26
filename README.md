# STM32F407 小车整合工程

当前唯一维护工程：**[F407_HAL_Template](new_project_f407/F407_HAL_Template/README.md)**。

STM32F407VET6 · Keil 5 / ARMCC 5 · STM32 HAL · ST-Link/SWD。
整合四轮速度PID、HWT101航向闭环、180°舵机、超声波、升降与横杆脉冲步进轴。水泵不参与运行，仅保留关断初始化。

用户于2026-10-26确认当前二进制数据包实机测试成功。本次整理以此为基线；清理后的编译与主机测试见工程内验证记录，不能替代重新烧录验证。

- [接线及资源分配](new_project_f407/F407_HAL_Template/docs/wiring.md)
- [串口二进制协议与测试包](new_project_f407/F407_HAL_Template/docs/protocol.md)
- [状态机](new_project_f407/F407_HAL_Template/docs/state-machine.md)
- [逐项测试记录](new_project_f407/F407_HAL_Template/docs/test-history.md)
- [复测步骤](new_project_f407/F407_HAL_Template/docs/test-guide.md)
- [原理图与硬件资料](new_project_f407/F407_HAL_Template/docs/hardware/README.md)

旧F103工程、临时资料、重复上游工程和阶段测试入口已移出工作目录；原有Git历史保留。清理前完整文件归档保存在本机工程目录外，详情见 [整理记录](new_project_f407/F407_HAL_Template/docs/cleanup.md)。
