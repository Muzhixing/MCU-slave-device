#!/usr/bin/env python3
"""Reproducible STM32F407VE GNU build from the Keil source list. No flash action."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

ap = argparse.ArgumentParser()
ap.add_argument("--source", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--toolchain", required=True, help="Directory containing arm-none-eabi-gcc")

args = ap.parse_args()
source, out, tc = Path(args.source).resolve(), Path(args.out).resolve(), Path(args.toolchain).resolve()
out.mkdir(parents=True, exist_ok=True)
gcc = str(tc / "arm-none-eabi-gcc")
version = subprocess.check_output([gcc, "--version"], text=True).splitlines()[0]
project = next((source / "MDK-ARM").glob("*.uvprojx"))
targets = ET.parse(project).findall("./Targets/Target")
target = targets[0]
files = []
for f in target.findall(".//FilePath"):
    value = f.text.replace("\\", "/")
    if value.endswith(".c"):
        files.append((source / "MDK-ARM" / value).resolve())
# Derive the vector order from the pinned ST startup, rather than guessing IRQ indices.
original = (source / "MDK-ARM/startup_stm32f407xx.s").read_text()
vector_start = original.index("__Vectors       DCD")
vector_text = original[vector_start:original.index("__Vectors_End\n", vector_start)]
vectors = re.findall(r"\bDCD\s+(\w+)", vector_text)
assert len(vectors) == 98, len(vectors)
vectors[0] = "_estack"
startup = '''.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb
.section .isr_vector,"a",%progbits
.global g_pfnVectors
g_pfnVectors:
''' + "".join(".word " + name + "\n" for name in vectors) + '''
.section .text.Reset_Handler,"ax",%progbits
.global Reset_Handler
.type Reset_Handler,%function
.thumb_func
Reset_Handler:
 ldr r0, =_sidata
 ldr r1, =_sdata
 ldr r2, =_edata
1: cmp r1, r2
 bcs 2f
 ldr r3, [r0], #4
 str r3, [r1], #4
 b 1b
2: ldr r1, =_sbss
 ldr r2, =_ebss
 movs r3, #0
3: cmp r1, r2
 bcs 4f
 str r3, [r1], #4
 b 3b
4: bl SystemInit
 bl main
 b HardFault_Handler
.size Reset_Handler, .-Reset_Handler
.section .text.Default_Handler,"ax",%progbits
.thumb_func
Default_Handler:
 b HardFault_Handler
'''
for name in sorted(set(vectors[2:]) - {"0"}):
    startup += ".weak " + name + "\n.thumb_set " + name + ",Default_Handler\n"
(out / "startup.S").write_text(startup)
(out / "stm32f407ve.ld").write_text('''ENTRY(Reset_Handler)
MEMORY { FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
         RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K }
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
 .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) } > FLASH
 .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); } > FLASH
 .ARM.extab : { *(.ARM.extab*) } > FLASH
 .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
 .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH
 .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH
 .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH
 _sidata = LOADADDR(.data);
 .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
 .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
 .heap (NOLOAD) : { . = ALIGN(8); end = .; _end = .; . += 0x800; } > RAM
 ASSERT(. < _estack - 0x1000, "RAM overflow into reserved 4K stack")
}
''')
flags = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard", "-Os", "-g3", "-std=c99",
         "-ffunction-sections", "-fdata-sections", "-Wall", "-Wextra", "-Wno-misleading-indentation",
         "-DUSE_HAL_DRIVER", "-DSTM32F407xx", "-DHSE_VALUE=8000000U"]
for path in ["User", "Hardware", "Drivers/STM32F4xx_HAL_Driver/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy", "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include"]:
    flags.extend(["-I", str(source / path)])
commands, logs, objects = [], [], []


def run(cmd):
    commands.append(cmd)
    process = subprocess.run(cmd, capture_output=True, text=True)
    logs.append(process.stdout + process.stderr)
    (out / "build.log").write_text("\n".join(logs))
    if process.returncode:
        print(logs[-1]); raise SystemExit(process.returncode)


for i, file in enumerate(files + [out / "startup.S"]):
    obj = out / (str(i) + "_" + file.stem + ".o")
    run([gcc, *flags, "-c", str(file), "-o", str(obj)])
    objects.append(str(obj))
elf = out / "f407_a75_driver_feedback.elf"
run([gcc, *flags, "-nostartfiles", "--specs=nano.specs", "--specs=nosys.specs", *objects,
     "-T", str(out / "stm32f407ve.ld"), "-Wl,--gc-sections,-Map=" + str(elf.with_suffix(".map")),
     "-Wl,--start-group", "-lc", "-lm", "-lgcc", "-Wl,--end-group", "-o", str(elf)])
for suffix, fmt in [(".hex", "ihex"), (".bin", "binary")]:
    run([str(tc / "arm-none-eabi-objcopy"), "-O", fmt, str(elf), str(elf.with_suffix(suffix))])
size = subprocess.check_output([str(tc / "arm-none-eabi-size"), str(elf)], text=True)
symbols = subprocess.check_output([str(tc / "arm-none-eabi-nm"), "--defined-only", str(elf)], text=True)
(out / "symbols.txt").write_text(symbols)
manifest = {"mode": "vehicle", "toolchain": version, "baseline": "e58f9dc333fe76bafdf414528d04ebd1a1d39924",
            "size": size, "build_commands": commands, "artifacts": {},
            "source_hashes": {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in source.rglob("*") if p.is_file() and p.suffix in (".c", ".h", ".inc", ".s", ".uvprojx")},
            "generated_hashes": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (out / "startup.S", out / "stm32f407ve.ld")}}
for path in [elf, elf.with_suffix(".hex"), elf.with_suffix(".bin")]:
    manifest["artifacts"][path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
(out / "build_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(size, json.dumps(manifest["artifacts"], indent=2))
