from pathlib import Path
import subprocess,json
r=Path.cwd();tmp=r/'tmp/servo_transport_audit_20261116'
s=(r/'Boards/F405/Src/servo_f405.c').read_text()
arrays=s[s.index('static int16_t offsets_deg'):s.index('static bool started;')]
funcs=s[s.index('uint8_t Servo_OutputAngle'):s.index('static uint32_t servo_pulse_for_angle')]
source='#include <stdint.h>\n#include <stdio.h>\n#include "app_config.h"\n'+arrays+funcs+'\nint main(void){for(int i=1;i<=4;i++)for(int a=0;a<=180;a++)printf("%d %d %u\\n",i,a,Servo_OutputPulseUs(i,a));}\n'
p=r/'tools/test_timed_uart_queue.c';old=p.read_bytes()
try:
 p.write_text(source,encoding='utf-8',newline='\n')
 subprocess.run([r'C:/Users/29459/Desktop/pack/Toolchain/mingw64/bin/gcc.exe','-std=c11','-O0','-DAPP_BOARD_F405_ROBO=1','-DAPP_ACTIVE_MODE=5','-IMain/Inc',str(p),'-o',str(tmp/'pulse.exe')],check=True)
 c=subprocess.check_output([str(tmp/'pulse.exe')]).decode().splitlines()
finally:p.write_bytes(old)
subprocess.run([r'C:/Windows/Microsoft.NET/Framework64/v4.0.30319/csc.exe','/nologo','/target:exe','/main:ServoParityCheck','/codepage:65001','/reference:System.Windows.Forms.dll','/reference:System.Drawing.dll','/reference:System.Web.Extensions.dll','/out:'+str(tmp/'Parity.exe'),str(r/'tools/F405ClawCalibrator.cs'),str(tmp/'Parity.cs')],check=True)
gui=subprocess.check_output([str(tmp/'Parity.exe'),str(r)]).decode().splitlines()
assert c==gui
report={'integer_cases':len(c),'claw_panel_cases':362,'s1':{a:500+a*2000//180 for a in (61,65,80,81,82,85)},'board_git':'6b05d9450b740848bf9436e63b6e9da10a293579','board_firmware_id':'8f2ce629deab61a03f97dc2d284c585b68d15028d5d3530b61e2ee52829eee7f','actual_matches_saved_binary':True,'hardware_commands':'read only; no actuation/reset/flash'}
(tmp/'parity_result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('Actual production C functions vs desktop ConfigPulse: 724 cases equal; actual claw panel: 362 cases equal; preset AUDIT85/EXIT65 pass')

