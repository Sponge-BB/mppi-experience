#!/usr/bin/python3
"""Archive this repair's explicit file group using the existing archive helper."""
import datetime,difflib,importlib.util,json,shutil,sys,tempfile
from pathlib import Path
ROOT=Path('/home/l/remani_ws');CASE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('archive',ROOT/'rollback/20261116-232420-snapshot-resync/evidence/archive_helper.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
name=sys.argv[1];config=json.loads((CASE/name/'version_config.json').read_text());state=json.loads((CASE/'version_state.json').read_text())
base=a.ROLL/state['baseline'];bm=json.loads((base/'manifest.json').read_text());before_all=json.loads((base/'files.json').read_text())
files=config['files'];before={p:before_all[p] for p in files if p in before_all};after=a.inventory(ROOT,files)
added=sorted(set(after)-set(before));deleted=sorted(set(before)-set(after));modified=sorted(p for p in set(before)&set(after) if before[p]!=after[p])
assert added or deleted or modified
v=datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+config['suffix'];dest=a.ROLL/v;dest.mkdir()
a.copy_entries(base/'files',dest/'before',before);a.copy_entries(ROOT,dest/'files',after)
a.archive(dest/'delta.tar.gz',dest/'files',after);a.write(dest/'files.json',after);a.write(dest/'before_files.json',before)
with tempfile.TemporaryDirectory(prefix='remani_repair_restore_') as td:
 stage=Path(td);a.copy_entries(dest/'before',stage,before)
 for p in deleted:(stage/p).unlink()
 a.copy_entries(dest/'files',stage,after);assert a.inventory(stage,files)==after
 for p in added:(stage/p).unlink()
 a.copy_entries(dest/'before',stage,before);assert a.inventory(stage,files)==before
patch=[]
for p in sorted(set(added+deleted+modified)):
 try:
  old=dest/'before'/p;new=dest/'files'/p
  lines1=old.read_text().splitlines(True) if old.is_file() else []
  lines2=new.read_text().splitlines(True) if new.is_file() else []
  patch+=difflib.unified_diff(lines1,lines2,fromfile='before/'+p,tofile='after/'+p)
 except UnicodeError:patch.append('Binary changed: '+p+'\n')
(dest/'changes.diff').write_text(''.join(patch));evidence=dest/'evidence';evidence.mkdir()
for f in config.get('evidence',[]):
 p=CASE/name/f;assert p.is_file() and p.stat().st_size<8*1024*1024;shutil.copy2(p,evidence/p.name)
shutil.copy2(__file__,evidence/'save_version.py');shutil.copy2(CASE/name/'version_config.json',evidence/'version_config.json')
parent=state['current_parent'];chain=bm['inherited_restore_chain']+[state['baseline']]+state['versions']
validation=json.loads((CASE/name/'validation_summary.json').read_text());assert validation['offline_passed'] and not validation['hardware_validation']
man={'version':v,'parent_version':parent,'complete_base':bm['complete_base'],'inherited_restore_chain':chain,'state':'OFFLINE_VERIFIED_HARDWARE_PENDING','purpose':config['purpose'],'files':files,'modified':modified,'added':added,'deleted':deleted,'validation':validation,'restore_verified':True,'boundary':'仅所列源码与部署产物，不含系统/外部SDK/硬件固件或控制器状态。未发实车指令，实车待验证。'}
a.write(dest/'manifest.json',man)
(dest/'RESTORE.txt').write_text('版本 '+v+'；父版本 '+parent+'。停止相关程序并保存现状后校验SHA256SUMS；在空目录重建完整基线和manifest.inherited_restore_chain，逐版应用增删改。\n本版本逆向恢复：删除新增文件 '+repr(added)+'；从before恢复全部文件并校验before_files.json（含源码/部署/权限/符号链接）。删除清单 '+repr(deleted)+' 也由before恢复。不得只解压覆盖；后续版本需先逆序处理；不使用旧build安装覆盖恢复程序。\n')
a.sums(dest);a.append_row(v,parent,man['state'],config['purpose']+'；正反向恢复校验通过；实车待验证')
state['versions'].append(v);state['current_parent']=v;a.write(CASE/'version_state.json',state)
print(json.dumps({'version':v,'parent':parent,'modified':modified,'added':added,'deleted':deleted},ensure_ascii=False,indent=2))
