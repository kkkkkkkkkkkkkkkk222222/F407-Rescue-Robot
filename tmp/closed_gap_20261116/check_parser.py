from pathlib import Path
import subprocess,json,sys
r=Path.cwd();sys.path.insert(0,str(r))
from tools.vision_protocol import mission_frame
c=json.loads((r/'tmp/closed_gap_20261116/upper_contract.json').read_text())
route=json.loads((r/'tmp/closed_gap_20261116/route_contract.json').read_text())
messages=c['examples']+[m for e in route['examples'] for m in e['messages']]
calls=[]
for m in messages:
 ctx=bytes.fromhex(m['context_hex']);cmd=bytes.fromhex(m['command_hex'])
 assert mission_frame(cmd[3],cmd[4],cmd[5],m['A'],m['B'],m['U'])==cmd
 wire=','.join(str(v) for v in ctx+cmd)
 calls.append('{ const uint8_t w[]={'+wire+'}; Vision_Init();Vision_ParseBytes(w,sizeof(w),100);VisionMissionCommand m=Vision_GetSnapshot().mission;assert(m.received&&m.context_valid&&m.payload_valid);assert(m.task_id==%d&&m.action_id==%d&&m.target_x_mm==%d&&m.target_y_mm==%d&&m.heading_cdeg==%d);}'%(m['task_id'],m['action_id'],m['A'],m['B'],m['U']))
source=r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
void BoardF405_RequestBeep(uint8_t n,bool l){(void)n;(void)l;}
void BoardF405_SetOperatorAlarm(bool e){(void)e;}
bool DebugTelemetry_RxByte(uint8_t b,bool s){(void)b;(void)s;return false;}
void TaskDiag_Event(const char*k,uint32_t a,uint32_t b,uint32_t c,uint32_t d,uint32_t e,uint32_t f){(void)k;(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;}
bool Uart_Send(const uint8_t *d,uint16_t n){(void)d;(void)n;return true;}
#include "../Main/Src/vision.c"
static void check(unsigned opcode,uint8_t flags,int a,int b,unsigned u,bool valid) {
 uint8_t wire[30],ctx[8]={0,7,0,1,0,0,0,0},payload[8]={opcode,flags,(uint8_t)(a>>8),(uint8_t)a,(uint8_t)(b>>8),(uint8_t)b,(uint8_t)(u>>8),(uint8_t)u};
 Vision_Init();vision_build_frame(wire,0x1d,1,ctx);vision_build_frame(wire+15,0x18,1,payload);
 Vision_ParseBytes(wire,30,100);VisionMissionCommand m=Vision_GetSnapshot().mission;
 assert(m.received&&m.context_valid&&m.payload_valid==valid);
}
int main(void){
CALLS
for(unsigned red=0;red<2;red++){
 unsigned f=red?9:1,u=red?9000:27000;
 check(4,f|0x24,0,11500,u,true);check(4,f|4,0,0,u,true);
 check(4,f|0x24,0,0,(u+18000)%36000,false);
 check(4,f|0x64,0,0,u,false);check(4,f|0x26,0,0,u,false);check(4,f|0x20,0,0,u,false);
 check(4,f|0x24,1,0,u,false);check(4,f|0x24,0,12000,u,false);
 check(3,f|0x70,100,0,12345,true);check(3,f|0x76,100,0,12345,true);
 check(3,f|0x36,100,0,u,false);check(3,f|0x72,100,0,u,false);
 check(3,f|0x76,-1,0,u,false);check(3,f|0x76,100,1,u,false);
 check(3,f|0x76,100,0,36000,false);check(5,f|0x24,0,0,u,false);
}
puts("6 new + 40 updated route paired frames: actual C parser and Python byte match; 32 parser boundary/legacy checks PASS");return 0;
}
'''.replace('CALLS','\n'.join(calls))
p=r/'tools/test_timed_uart_queue.c';old=p.read_bytes()
try:
 p.write_text(source,encoding='utf-8',newline='\n')
 exe=r/'tmp/closed_gap_20261116/parser.exe'
 subprocess.run([r'C:/Users/29459/Desktop/pack/Toolchain/mingw64/bin/gcc.exe','-std=c11','-O0','-DAPP_BOARD_F405_ROBO=1','-DAPP_ACTIVE_MODE=5','-Itools/test_support','-IMain/Inc',str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
finally:p.write_bytes(old)

