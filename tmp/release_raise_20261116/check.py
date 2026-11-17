from pathlib import Path
import subprocess
r=Path.cwd();p=r/'tools/test_audit_handshake.c';original=p.read_bytes()
try:
 text=original.decode('utf-8').replace('\r\n','\n')
 start=text.index('static void test_normalrun_start_and_delivery_exit(void)\n{')
 body=text.index('  prepare();\n  initial_claw_ready = true;\n  test_vision.config_ready = true;\n  Lift_SetTravelPosition();',start)
 text=text[:start]+'static void test_normalrun_start_and_delivery_exit(void)\n{\n'+text[body:]
 text=text.replace('int main(void)\n{','int main(void)\n{\n test_normalrun_start_and_delivery_exit();puts("Existing delivery-exit check PASS");return 0;\n',1)
 p.write_text(text,encoding='utf-8',newline='\n')
 exe=r/'tmp/release_raise_20261116/check.exe'
 subprocess.run([r'C:/Users/29459/Desktop/pack/Toolchain/mingw64/bin/gcc.exe','-std=c11','-O0','-DAPP_BOARD_F405_ROBO=1','-DAPP_ACTIVE_MODE=5','-DAPP_NORMALDEBUG_MOTION_TEST=0','-Itools/test_support','-IMain/Inc',str(p),'Main/Src/pid.c','-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
finally:p.write_bytes(original)

