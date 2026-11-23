# 四轮编码器滤波与悬空验收 · 2026-11-23

四路编码器输入滤波从6改为15后，已烧录固件在悬空8%速度目标下完成前进、后退各120秒，无系统、电机或UART错误；用户确认“两个方向全程平滑，没有恶化”。此结果支持将该版本保存为四轮低速调试基线。

## 机制与改动

此前各轮低速卡顿节奏不同，加速后减轻。B/TIM3进行滤波6→15→6的可逆对照时，用户观察到中间段明显平滑。输入数字滤波要求信号连续稳定后才接受边沿，可抑制短脉冲干扰；本次将同一配置用于A/TIM2、B/TIM3、C/TIM4和D/TIM8的两个编码器输入。计数方向、换算、PI参数、PWM上限、故障处理、串口协议及ABC逻辑均沿用基线。

运行态SWD曾产生异常零值和不一致读回，早期逐轮PWM/反向计数统计只保留为诊断线索，不作为本发布的定量改善指标。现场对照与后续完整固件测试支持平滑性改善，尚未确定电气干扰的物理来源。

## 核对与验收

|检查|条件与结果|证据|
|---|---|---|
|编码器轮位|PWM均为0时依次手转：D左前、B右前、A右后、C左后；PWM与物理轮位尚未独立逐路确认|[映射记录](validation/20261123-all-encoder-filter/encoder-wheel-mapping.json)|
|构建及映像|GNU Arm10.3.1完整编译链接；93个源码输入哈希匹配；HEX校验与BIN一致|[发布文件](../firmware/20261123-all-encoder-filter/README.md)、[发布复核](validation/20261123-all-encoder-filter/release-review.json)|
|烧录与重启|OpenOCD verify通过，暂停状态读回20136字节与BIN相同；重启后四路CCMR1=0xF1F1，PWM均为0|[烧录结果](validation/20261123-all-encoder-filter/flash-result.json)、[烧录日志](validation/20261123-all-encoder-filter/program.log)|
|10%悬空正反向|后退9.794秒、前进9.796秒，各39帧运行回传，无故障，均确认禁用停车；用户确认两个方向四轮都转、明显平滑|[现场确认](validation/20261123-all-encoder-filter/motion-10-percent-acceptance.json)|
|8%悬空持续运行|前进120.001秒、后退120.003秒，各480帧运行回传；系统/电机/UART错误均0，ABC保持A90/B0/C0|[分析](validation/20261123-all-encoder-filter/endurance-8-percent-analysis.json)、[现场确认](validation/20261123-all-encoder-filter/endurance-8-percent-acceptance.json)|
|主机回归|包编解码、独立零点、运动状态机、禁用停车、航向丢失与恢复、实际USART2解析驱动测试通过|`python3 tests/run.py`|

8%与10%均是协议速度目标字段，不是PWM占空比或标定后的线速度。持续测试每段仅发一帧运动命令和一帧禁用命令，最终状态为disabled、命令计数10、故障0。两段均超过此前约85秒故障时间点；平滑性由现场观察确认，原协议回传不提供逐轮转速误差。

## 调试读回与备份

烧录前在MCU暂停、PWM为0的条件下取得两份完全一致的512KiB备份，SHA-256为 `7841f900fe332a5511b37610e0c53879caca482cd889f376837220c07abee6eb`，完整备份保留在现场证据目录。此前运行中读取的备份有异常，不能作为恢复源。

新固件的暂停态校验通过，运行态SWD读取仍出现异常；[调试日志](validation/20261123-all-encoder-filter/postflash-debug-check.log) 中的 `running_filter_mismatch` 对应这一未解决问题。DBGMCU_CR原本为7，设置DBG_SLEEP未改变结果，不能据此认定WFI或调试睡眠为根因。运动验收采用串口状态和现场反馈。

## 本阶段认识

建立的认识：该映像已在悬空8%目标、正反向各120秒的范围内完成平滑运行与停车验证，可作为下一阶段基线。

关键证据：源文件与映像校验、暂停读回一致、重启后四路滤波设置、960帧无故障运行回传及用户全程观察。

下一项最有价值的工作：转向负载验证前补齐MCU失联停车，并逐路确认PWM输出与编码器配对，再验证横移/转向。具体待办集中于 [后续验证](four-wheel-followups.md)。
