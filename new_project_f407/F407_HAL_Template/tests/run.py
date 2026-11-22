"""Host regressions; original protocol plus real-parser heading recovery. No device access."""
from pathlib import Path
import tempfile, shutil, subprocess, os
root=Path(__file__).resolve().parent
hardware=root.parent/'Hardware'
compiler=os.environ.get('CC','gcc')
for test in ('test_control.c','test_heading_recovery.c'):
    with tempfile.TemporaryDirectory(prefix='f407-tests-') as temp:
        build=Path(temp)
        for f in (root/'mocks').glob('*.h'): shutil.copy2(f,build/f.name)
        for name in ('UpperPacket','AxisControl','SystemControl','StepperPulse','HWT101_Parser'):
            for ext in ('c','h'): shutil.copy2(hardware/(name+'.'+ext),build/(name+'.'+ext))
        shutil.copy2(root/test,build/test)
        exe=build/('test.exe' if os.name=='nt' else 'test')
        flags=['-std=c99','-Wall','-Wextra','-Werror','-I',str(build)]
        if os.environ.get('SANITIZE')=='1': flags+=['-fsanitize=address,undefined','-fno-omit-frame-pointer','-g']
        subprocess.run([compiler,*flags,*map(str,sorted(build.glob('*.c'))),'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

with tempfile.TemporaryDirectory(prefix='f407-driver-tests-') as temp:
    build=Path(temp)
    shutil.copy2(root/'heading_driver_mocks/main.h',build/'main.h')
    for name in ('HWT101','HWT101_Parser'):
        for ext in ('c','h'): shutil.copy2(hardware/(name+'.'+ext),build/(name+'.'+ext))
    shutil.copy2(root/'test_heading_driver.c',build/'test_heading_driver.c')
    exe=build/('driver.exe' if os.name=='nt' else 'driver')
    flags=['-std=c99','-Wall','-Wextra','-Werror','-I',str(build)]
    if os.environ.get('SANITIZE')=='1':flags+=['-fsanitize=address,undefined','-fno-omit-frame-pointer','-g']
    subprocess.run([compiler,*flags,*map(str,sorted(build.glob('*.c'))),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
