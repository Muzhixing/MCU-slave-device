# 已烧录发布：A73 上电回正

目标STM32F407VET6、HSE8MHz，源码基线 `886b45492ef07981a9f60626992f669658a65448`。唯一运行源码改动是将 `SERVO_RESET_TENTHS` 从900改为730；四轮滤波15及其余控制逻辑保留。现场75°略偏左，调整到73°后用户确认居中，烧录复位后再次确认仍居中。

BIN为20136字节，装载地址0x08000000；SHA-256：`a71097d52748721a7e3132516278cfeffbf3f5e5388ccb958c66b6f1a08b1e41`。OpenOCD校验和暂停读回一致。HEX与BIN内容一致，匹配的ELF/MAP、生成的启动汇编/链接脚本、构建清单及日志均在本目录，校验见 `SHA256SUMS`。

构建使用GNU Arm10.3.1；从工程根目录重建：

```sh
python3 tools/build.py --source . --out /tmp/f407-a73-build --toolchain /path/to/toolchain/bin --name f407_a73_encoder_filter --baseline 886b45492ef07981a9f60626992f669658a65448
```

原始 `build_manifest.json` 中 `hardware_validated=false` 是构建当时的状态；后续烧录与验收见 [启动零位](../../docs/startup-zero-20261123.md)。不同构建不能自动继承本次实机验证。

RK3588必须配套A73启动配置。B/C仍由驱动器自主回零，上位机在已确认的共同重启约定下等待8秒后登记；本固件没有新增回零完成传感器或CAN反馈。测试结束保持A73/B0/C0、禁用、故障0。
