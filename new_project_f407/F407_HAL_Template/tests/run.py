"""Host state-machine regression tests; GCC required. No hardware flashing."""
from pathlib import Path
import tempfile,shutil,subprocess,os
root=Path(__file__).resolve().parent
hardware=root.parent/'Hardware'
with tempfile.TemporaryDirectory(prefix='f407-tests-') as temp:
    build=Path(temp)
    for f in (root/'mocks').glob('*.h'): shutil.copy2(f,build/f.name)
    for name in ['UpperPacket','AxisControl','SystemControl','StepperPulse']:
        for ext in ['c','h']: shutil.copy2(hardware/(name+'.'+ext),build/(name+'.'+ext))
    shutil.copy2(hardware/'HWT101_Parser.h',build/'HWT101_Parser.h')
    shutil.copy2(root/'test_control.c',build/'test_control.c')
    exe=build/('test.exe' if os.name=='nt' else 'test')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(build),*[str(f) for f in build.glob('*.c')],'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
