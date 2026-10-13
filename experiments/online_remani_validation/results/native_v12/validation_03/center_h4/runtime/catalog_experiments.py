#!/usr/bin/env python3
"""Non-destructive index: each original experiment gets its own GIF folder."""
import argparse
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--experiments',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    root=a.experiments.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    groups={}
    for project in sorted(root.iterdir()):
        results=project/'results'
        if not results.is_dir():continue
        for gif in sorted(results.rglob('*.gif')):
            if gif.is_symlink():continue
            rel=gif.relative_to(root);key=rel.parent
            groups.setdefault(key,[]).append(gif)
    lines=['# 实验 GIF 分类索引','',
           '此目录只建立链接，不移动、不覆盖原始实验、失败记录或构建。各实验的 GIF 单独存放。',
           'V6、V7 等版本按原目录分别索引。validation 为对照验证，development 为开发调试；请阅读各自结论，目录名不代表已通过实车验收。',
           '图中实验范围以原目录说明为准：局部固定参考、完整 REMANI 数值仿真、历史 CBF 试验不可混为一谈。','']
    for key,files in groups.items():
        # Keep original hierarchy so identical GIF names cannot collide.
        dest=out/key;dest.mkdir(parents=True)
        origin=files[0].parent.parent if files[0].parent.name=='gif' else files[0].parent
        (dest/'raw_experiment').symlink_to(origin,target_is_directory=True)
        for gif in files:(dest/gif.name).symlink_to(gif)
        lines.append(f'## {key}\n')
        for gif in files:lines.append(f'- [{gif.name}]({(key/gif.name).as_posix()})')
        lines.append('')
    (out/'README.md').write_text('\n'.join(lines)+'\n');print(out/'README.md',len(groups),'GIF groups')

if __name__=='__main__':main()
