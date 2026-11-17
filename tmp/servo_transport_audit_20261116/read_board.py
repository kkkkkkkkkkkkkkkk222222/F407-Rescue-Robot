import socket,json
from pathlib import Path
root=Path.cwd()/'tmp/servo_transport_audit_20261116'
commands=['servo_readall','f405_servo_table','read_memory 0x40023800 32 3','read_memory 0x40000828 32 2',f'dump_image {root.as_posix()}/actual_flash.bin 0x08000000 0x30000']
out=[]
with socket.create_connection(('127.0.0.1',60924),timeout=4) as s:
 s.settimeout(20)
 for c in commands:
  s.sendall(c.encode()+b'\x1a');data=b''
  while b'\x1a' not in data:
   part=s.recv(65536)
   if not part:raise RuntimeError('OpenOCD disconnected')
   data+=part
  text=data.split(b'\x1a')[0].decode(errors='replace');out.append({'command':c,'result':text});print(c,':',text)
(root/'readback.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')

