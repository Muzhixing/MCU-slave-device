#!/usr/bin/env python3
"""Verify linked vectors, A reset data, Intel HEX parity, and retained fixes."""
import argparse, hashlib, json, re, struct, subprocess, tempfile
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--build',required=True);ap.add_argument('--toolchain',required=True)
a=ap.parse_args();build=Path(a.build).resolve();tc=Path(a.toolchain).resolve();root=Path(__file__).resolve().parents[1]
elf=build/'f407_a75_driver_feedback.elf';raw=elf.with_suffix('.bin').read_bytes()
nm=subprocess.check_output([str(tc/'arm-none-eabi-nm'),'-a',str(elf)],text=True)
symbols={}
for line in nm.splitlines():
    fields=line.split()
    if len(fields)==3:
        try:symbols[fields[2]]=int(fields[0],16)
        except ValueError:pass
source=(root/'MDK-ARM/startup_stm32f407xx.s').read_text();begin=source.index('__Vectors       DCD')
vectors=re.findall(r'\bDCD\s+(\w+)',source[begin:source.index('__Vectors_End\n',begin)])
assert len(vectors)==98
actual=struct.unpack('<98I',raw[:392]);assert actual[0]==0x20020000
for name,value in zip(vectors[1:],actual[1:]):
    assert value==(0 if name=='0' else symbols[name]|1),(name,hex(value))
for name in ('DriverCAN_Init','DriverCAN_Poll','AxisFeedback_Receive','HWT101_Recover','MotorControl_Update'):
    assert name in symbols,name
# Initialized Servo.c data symbols are emitted as .data.current / .data.target.
# Use the object (unique static names) rather than similarly named linked symbols.
servo=next(build.glob('*_Servo.o'))
with tempfile.TemporaryDirectory() as temp:
    for name in ('current','target'):
        path=Path(temp)/name
        subprocess.run([str(tc/'arm-none-eabi-objcopy'),'--dump-section',f'.data.{name}={path}',str(servo)],check=True)
        assert struct.unpack('<H',path.read_bytes())[0]==750,name
    converted=Path(temp)/'hex.bin'
    subprocess.run([str(tc/'arm-none-eabi-objcopy'),'-I','ihex','-O','binary',str(elf.with_suffix('.hex')),str(converted)],check=True)
    assert converted.read_bytes()==raw
assert '#define SERVO_RESET_TENTHS 750U' in (root/'Hardware/Servo.h').read_text()
# Verify the preserved wheel direction and heading receive implementations byte for byte.
retained=[]
for name in ('MotorControl.c','MotorControl.h','motor.c','motor.h','HWT101.c','HWT101.h','HWT101_Parser.c','HWT101_Parser.h'):
    path=root/'Hardware'/name
    relative='new_project_f407/F407_HAL_Template/Hardware/'+name
    old=subprocess.check_output(['git','show','e58f9dc:'+relative],cwd=root)
    assert old==path.read_bytes(),name
    retained.append(name)
manifest=json.loads((build/'build_manifest.json').read_text())
for name,digest in manifest['artifacts'].items():assert hashlib.sha256((build/name).read_bytes()).hexdigest()==digest
report={'vectors_checked':98,'initial_stack':'0x20020000','flash_base':'0x08000000','servo_initialized_tenths':750,'hex_bin_identical':True,'preserved_sources':retained,'hardware_tested':False}
(build/'artifact_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
