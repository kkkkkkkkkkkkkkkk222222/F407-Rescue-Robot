from pathlib import Path
import subprocess,os
r=Path.cwd();tmp=r/'tmp/servo_transport_audit_20261116'
def git(*args,env=None):return subprocess.check_output(['git','-c','safe.directory='+str(r),*args],env=env).decode('utf-8')
before=(tmp/'Task.before.c').read_text()
base=git('show','HEAD:Main/Src/Task.c').replace('\r\n','\n')
old='  tracking_tick_ms = 0U;\n\n  \n  approach_report_generation = 0U;'
new='  tracking_tick_ms = 0U;\n  approach_report_generation = 0U;'
assert before.count(old)==1 and before.replace(old,new)==base
current=(r/'Main/Src/Task.c').read_text();assert current.count(old)==1
(tmp/'Task.commit.c').write_text(current.replace(old,new),encoding='utf-8',newline='\n')
files=['Main/Src/DebugConsole.c','tools/F405ClawCalibrator.cs','tools/F405ClawSweep.ini','tools/Start-F405ClawCalibrator.ps1','tools/Test-F405ClawSweep.ps1','tools/Update-F405SerialGuide.ps1','tools/test_audit_handshake.c','docs/F405_SERIAL_GUIDE.txt','docs/F405_CLAW_CALIBRATION.md','docs/F405_SERVO_TRANSPORT_AUDIT_20261116.md','docs/CURRENT_STATUS.md','docs/CROSS_REPO_REQUESTS.md']
env=os.environ.copy();env['GIT_INDEX_FILE']=str(tmp/'commit.index')
git('read-tree','HEAD',env=env);git('add','--',*files,env=env)
blob=git('hash-object','-w','--path=Main/Src/Task.c',str(tmp/'Task.commit.c')).strip()
git('update-index','--add','--cacheinfo','100644,'+blob+',Main/Src/Task.c',env=env)
git('diff','--cached','--check',env=env)
print(git('diff','--cached','--stat',env=env))
print(git('diff','--cached','--','Main/Src/Task.c','tools/F405ClawCalibrator.cs','tools/Test-F405ClawSweep.ps1',env=env))

