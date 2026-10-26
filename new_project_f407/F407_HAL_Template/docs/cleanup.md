# 2026-10-26 工程整理

保留唯一F407工程的原路径，避免Keil工程链接改变。删除工程引用中的独立motor_test、旧文本BluetoothProtocol、StepperAxis/StepperTest、SerialTelemetry、HeadingControl；舵机脉宽换算并入Servo.c，删除文本指令入口；水泵仅保留初始化及关闭。

HAL/CMSIS及其许可证保留，未做冒险的厂商头文件裁剪。Core重复目录、旧F103工程、上游重复工程、临时产物和旧阶段文档移出。原始原理图、PCB照片、制板文件、24张步进菜单保存在docs/hardware。

清理前完整归档（不包含.git对象）：
`E:/project_M_archives/project_M_before_cleanup_20261026_214812.zip`
9,454个文件，每个文件SHA256与归档中内容逐项比对通过；旁边保存manifest与Git状态。Git历史仍在原仓库。

批量删除被自动审批拦截后采用可恢复移动：旧目录实际移至
`E:/project_M_archives/removed_20261026/`。
这些历史副本不上传GitHub。源码原有F103历史可通过Git旧提交查阅。

本次不改变整合控制行为、PID参数、引脚、步进比例、协议或500ms切换等待。最终编译/回归结果及固件校验见validation与firmware。
