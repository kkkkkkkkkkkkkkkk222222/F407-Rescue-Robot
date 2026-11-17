from pathlib import Path
import os,subprocess
r=Path.cwd();tmp=r/'tmp/release_raise_20261116'
def git(*args,env=None):return subprocess.check_output(['git','-c','safe.directory='+str(r),*args],env=env).decode('utf-8')
current=(r/'Main/Src/Task.c').read_text(encoding='utf-8')
old='  tracking_tick_ms = 0U;\n\n  \n  approach_report_generation = 0U;'
new='  tracking_tick_ms = 0U;\n  approach_report_generation = 0U;'
assert current.count(old)==1
(tmp/'Task.commit.c').write_text(current.replace(old,new),encoding='utf-8',newline='\n')
files=['Main/Inc/app_config.h','Main/Inc/mechanism.h','Main/Src/mechanism.c','docs/CURRENT_STATUS.md','docs/F405_CLAW_CALIBRATION.md','tools/test_audit_handshake.c']
env=os.environ.copy();env['GIT_INDEX_FILE']=str(tmp/'commit.index')
git('read-tree','HEAD',env=env);git('add','--',*files,env=env)
blob=git('hash-object','-w','--path=Main/Src/Task.c',str(tmp/'Task.commit.c')).strip()
git('update-index','--add','--cacheinfo','100644,'+blob+',Main/Src/Task.c',env=env)
git('diff','--cached','--check',env=env)
print(git('diff','--cached','--stat',env=env))
print(git('diff','--cached','--','Main/Src/Task.c','tools/test_audit_handshake.c',env=env))

