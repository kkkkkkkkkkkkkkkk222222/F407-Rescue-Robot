from pathlib import Path
import os,subprocess
r=Path.cwd();tmp=r/'tmp/closed_gap_20261116'
def git(*args,env=None):
 return subprocess.check_output(['git','-c','safe.directory='+str(r),*args],env=env).decode('utf-8')
base=git('show','HEAD:Main/Src/Task.c').replace('\r\n','\n')
before=(tmp/'Task.before.c').read_text(encoding='utf-8')
old='  tracking_tick_ms = 0U;\n\n  \n  approach_report_generation = 0U;'
new='  tracking_tick_ms = 0U;\n  approach_report_generation = 0U;'
assert before.count(old)==1 and before.replace(old,new)==base
current=(r/'Main/Src/Task.c').read_text(encoding='utf-8')
assert current.count(old)==1
clean=current.replace(old,new)
(tmp/'Task.commit.c').write_text(clean,encoding='utf-8',newline='\n')
files=['Main/Inc/vision.h','Main/Src/vision.c','tools/vision_protocol.py','tools/test_vision_protocol.py','docs/CURRENT_STATUS.md','docs/CROSS_REPO_REQUESTS.md','docs/contracts/f405_two_station_tail_return_20261116.json','docs/contracts/f405_closed_gap_zone_cancel_20261116.json']
env=os.environ.copy();env['GIT_INDEX_FILE']=str(tmp/'commit.index')
git('read-tree','HEAD',env=env);git('add','--',*files,env=env)
blob=git('hash-object','-w','--path=Main/Src/Task.c',str(tmp/'Task.commit.c')).strip()
git('update-index','--add','--cacheinfo','100644,'+blob+',Main/Src/Task.c',env=env)
assert git('show',':Main/Src/Task.c',env=env).replace('\r\n','\n')==clean
git('diff','--cached','--check',env=env)
print(git('diff','--cached','--stat',env=env))
print(git('diff','--cached','--','Main/Src/Task.c',env=env))



