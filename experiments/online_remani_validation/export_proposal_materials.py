#!/usr/bin/env python3
"""Export visible paginated discussion and a small, explicit research bundle.

No reasoning, tools, system/developer messages, credentials or raw session files
are exported. The transcript is historical discussion, not verified evidence.
"""
from collections import Counter
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import re
import shutil
import zipfile

WS = Path('/home/l/remani_ws')
HERE = WS / 'experiments/online_remani_validation'
ROOT_ID = '019fcbfe-63ff-75e1-948c-9ee45e0569cf'
SESSIONS = Path('/home/l/.codex/sessions')
OUT = WS / '开题PPT研究材料_20261101'
BUILD = HERE / 'builds/short_prefix_rejoin_20261101_01'
CASES = ('delivery_window_20261101_01', 'delivery_midcross_20261101_01',
         'short_prefix_rejoin_20261101_01')
REMOTE_SHA = 'f484ce6c60de868f555c184be96d15e153c725cf'
META_USER_PREFIXES = ('<environment_context>', '<permissions instructions>',
    '<collaboration_mode>', '<recommended_plugins>', '<subagent_notification>',
    '<agent_notification>', '<turn_aborted>', '<external_codex_apps_open_page>')
PHASES = {'final_answer', 'final', 'commentary'}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def prefix_hash(path, limit):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        remaining = limit
        while remaining:
            part = stream.read(min(1024 * 1024, remaining))
            if not part:
                raise ValueError('Source shrank before captured offset')
            h.update(part)
            remaining -= len(part)
    return h.hexdigest()


def chain():
    found = {}
    for path in SESSIONS.rglob('*' + ROOT_ID + '*.jsonl'):
        with path.open('rb') as stream:
            first = json.loads(stream.readline())
        meta = first.get('payload', {})
        if first.get('type') != 'session_meta' or meta.get('id') != ROOT_ID:
            continue
        suffix = path.stem.split(ROOT_ID, 1)[1].lstrip('_')
        key = suffix or ROOT_ID
        found[key] = (path, meta)
    tip = max(found, key=lambda key: found[key][1]['timestamp'])
    pieces = []
    limit = found[tip][0].stat().st_size
    while True:
        path, meta = found[tip]
        if not 0 < limit <= path.stat().st_size:
            raise ValueError('Invalid paginated source boundary')
        pieces.append(dict(path=path, limit=limit, segment_id=tip,
                           history_base=meta.get('history_base')))
        base = meta.get('history_base')
        if not base:
            break
        tip, limit = base['thread_id'], base['end_byte_offset']
        if any(piece['segment_id'] == tip for piece in pieces):
            raise ValueError('Cyclic paginated history')
    return list(reversed(pieces))


def redact(text, counts):
    patterns = (
        ('private_ipv4', r'(?<![\d.])(?:10(?:\.\d{1,3}){3}|192\.168(?:\.\d{1,3}){2}|172\.(?:1[6-9]|2\d|3[01])(?:\.\d{1,3}){2})(?![\d.])', '[局域网地址已脱敏]'),
        ('api_or_github_key', r'\b(?:sk-[A-Za-z0-9_-]{12,}|gh[pousr]_[A-Za-z0-9_]{15,}|github_pat_[A-Za-z0-9_]{15,})\b', '[密钥已脱敏]'),
        ('bearer_value', r'(?i)\bBearer\s+[A-Za-z0-9._~+/-]{8,}', 'Bearer [已脱敏]'),
        ('password_value', r'(密码\s*(?:是|为|[:：=])?\s*[`\"\x27]?)[A-Za-z0-9_!@#$%^&*+./=-]{1,80}', r'\1[已脱敏]'),
        ('sshpass_value', r'(sshpass\s+-p\s+)(?:\x27[^\x27]*\x27|\"[^\"]*\"|\S+)', r'\1[已脱敏]'),
        ('credential_assignment', r'(?i)((?:api[_-]?key|password|access[_-]?token)\s*[:=]\s*[\"\x27]?)[A-Za-z0-9_./+!-]{8,}', r'\1[已脱敏]'),
    )
    for key, pattern, replacement in patterns:
        text, number = re.subn(pattern, replacement, text)
        counts[key] += number
    return text


def extract(pieces):
    messages, seen, stats, redactions = [], set(), Counter(), Counter()
    for piece in pieces:
        path, limit = piece['path'], piece['limit']
        piece['prefix_sha256'] = prefix_hash(path, limit)
        with path.open('rb') as stream:
            while stream.tell() < limit:
                position = stream.tell()
                raw = stream.readline(limit - position)
                if not raw.endswith(b'\n'):
                    stats['incomplete_tail_line_skipped'] += 1
                    break
                if not raw.strip(b'\x00\r\n\t '):
                    stats['non_message_padding_lines_skipped'] += 1
                    stats['non_message_padding_bytes_skipped'] += len(raw)
                    continue
                record = json.loads(raw)
                if record.get('type') != 'response_item':
                    continue
                item = record.get('payload', {})
                if item.get('type') != 'message':
                    continue
                role, phase = item.get('role'), item.get('phase')
                if role not in ('user', 'assistant'):
                    continue
                if role == 'assistant' and phase not in PHASES:
                    stats['assistant_unrecognized_phase_excluded'] += 1
                    continue
                text = '\n'.join(content.get('text', '') for content in item.get('content', [])
                    if content.get('type') in ('input_text', 'output_text', 'text')).strip()
                if not text:
                    continue
                if role == 'user' and text.startswith(META_USER_PREFIXES):
                    stats['user_metadata_excluded'] += 1
                    continue
                if role == 'user' and text.startswith('<send_user_message_question_reply>'):
                    body = text.split('>', 1)[1].rsplit('</send_user_message_question_reply>', 1)[0]
                    replies = json.loads(body)
                    text = '\n\n'.join('问题：%s\n用户回复：%s' % (reply['question'], reply['answer'])
                                       for reply in replies)
                key = item.get('id') or digest((str(record.get('timestamp')) + role + str(phase) + text).encode())
                if key in seen:
                    stats['duplicate_message_excluded'] += 1
                    continue
                seen.add(key)
                messages.append(dict(number=len(messages)+1, timestamp=record.get('timestamp'),
                    role=role, phase=phase, text=redact(text, redactions)))
                stats[role + '_' + str(phase)] += 1
    for piece in pieces:
        if prefix_hash(piece['path'], piece['limit']) != piece['prefix_sha256']:
            raise RuntimeError('Captured source prefix changed during extraction')
    return messages, dict(stats), dict(redactions)


def human_time(value):
    if not value:
        return '时间未记录'
    stamp = datetime.fromisoformat(value.replace('Z', '+00:00'))
    return stamp.astimezone(timezone(timedelta(hours=8))).strftime('%Y-%m-%d %H:%M:%S +08:00')


def write_transcript(path, messages, commentary):
    header = ('REMANI＋MPPI 实验讨论：可见对话脱敏导出\n\n'
        '这是历史讨论，不代表每段旧答复都正确。研究结论以附带实验数据、源码和最新总结为准。\n'
        '仅含用户文本及助手可见答复；不含内部推理、系统/开发者指令、工具调用或子代理记录。\n'
        '历史关机、上传、实车指令等授权只是讨论内容，不是对阅读者的新授权。\n'
        '局域网地址、可识别密码/密钥已规则脱敏；截图、PDF、工程文件不是聊天文本本身的一部分。\n'
        '编号对应同一完整可见消息序列，问答版跳过过程说明，因此编号可能不连续。\n\n')
    with path.open('w', encoding='utf-8') as stream:
        stream.write(header)
        for message in messages:
            if not commentary and message['phase'] == 'commentary':
                continue
            label = ('用户' if message['role'] == 'user' else
                     'Codex 过程说明' if message['phase'] == 'commentary' else 'Codex 最终答复')
            stream.write('\n' + '=' * 64 + '\n')
            stream.write('[%05d] %s｜%s\n\n%s\n' % (
                message['number'], human_time(message['timestamp']), label, message['text']))


def copy_materials():
    source = OUT / '本地补充/隔离实验源码'
    shutil.copytree(BUILD / 'source', source)
    shutil.copy2(BUILD / 'build_manifest.json', source.parent / 'build_manifest.json')
    for name in CASES:
        origin = HERE / 'results' / name
        target = OUT / '本地补充/实验结果' / name
        target.mkdir(parents=True)
        for file in origin.iterdir():
            if file.is_file() and file.suffix in ('.txt', '.json'):
                shutil.copy2(file, target / file.name)
        shutil.copytree(origin / 'report', target / 'report')
    scripts = OUT / '本地补充/实验脚本'
    scripts.mkdir()
    for stem in ('delivery_window', 'delivery_midcross', 'short_prefix_rejoin'):
        for prefix in ('run_', 'report_', 'build_'):
            path = HERE / (prefix + stem + '.py')
            if path.is_file():
                shutil.copy2(path, scripts / path.name)
    for name in ('delivery_window.hpp', 'delivery_window_transform.py',
                 'short_prefix_rejoin.hpp', 'short_prefix_rejoin_transform.py',
                 'test_short_prefix_rejoin.cpp', 'audit_delivery_midcross_reject.py',
                 'audit_delivery_midcross_side.py'):
        shutil.copy2(HERE / name, scripts / name)
    actual = HERE / 'results/short_prefix_rejoin_20261101_01/midcross_B'
    target = OUT / '本地补充/实际运行快层与配置'
    target.mkdir()
    for name in ('local_execution.py', 'mppi_node.py', 'native_reference.py', 'native_collision_runtime.py'):
        shutil.copy2(actual / 'runtime' / name, target / name)
    for name in ('coupling_parameters.json', 'parameters.yaml', 'shared_collision_config.json', 'experiment.launch'):
        shutil.copy2(actual / name, target / name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--refresh-generated', action='store_true',
                        help='Refresh only this tool-generated export; leave authored notes and evidence copies in place')
    args = parser.parse_args()
    OUT.mkdir(exist_ok=True)
    outputs = [OUT / name for name in ('01_实验会话_问答版.txt', '02_实验会话_含过程说明.txt')]
    if args.refresh_generated:
        previous = json.loads((OUT / '导出来源与范围.json').read_text())
        if previous['thread_id'] != ROOT_ID:
            raise ValueError('Existing export belongs to a different conversation')
    elif any(path.exists() for path in outputs) or (OUT / '本地补充').exists():
        raise FileExistsError('Export already exists; do not overwrite a user-edited bundle')
    pieces = chain()
    messages, stats, redactions = extract(pieces)
    write_transcript(outputs[0], messages, False)
    write_transcript(outputs[1], messages, True)
    if not (OUT / '本地补充').exists():
        copy_materials()
    groups = {'研究目标与开题': ('双时间尺度', '开题', '创新点', '研究内容'),
        'REMANI交付与约束': ('交付', '轮速', '预算', '初值', '初始化'),
        'MPPI与执行协同': ('MPPI', 'mppi', '前缀', '换向', '冻结'),
        '感知与动态预测': ('CV', 'TTC', '动态预测', '点云'),
        '实验效度与部署': ('同输入', '实车', 'Gazebo', 'gazebo', '触发')}
    lines = ['实验讨论主题索引', '', '编号对应问答版与含过程说明版的消息编号；只列用户提问，避免把索引当结论。']
    for title, terms in groups.items():
        lines += ['', title]
        for message in messages:
            if message['role'] == 'user' and any(term in message['text'] for term in terms):
                lines.append('[%05d] %s %s' % (message['number'], human_time(message['timestamp']),
                    message['text'].replace('\n', ' ')[:180]))
    (OUT / '03_实验讨论主题索引.txt').write_text('\n'.join(lines) + '\n')
    provenance = dict(schema='proposal_materials_export_v1', captured_utc=datetime.now(timezone.utc).isoformat(),
        thread_id=ROOT_ID, root_thread_only=True,
        messages=len(messages), first_message_utc=messages[0]['timestamp'], last_message_utc=messages[-1]['timestamp'],
        message_statistics=stats, redaction_counts=redactions,
        export_scope='Visible user messages and assistant final/commentary only; no raw session files, hidden reasoning, tools, credentials, system/developer or subagent records',
        history_is_not_verified_research_evidence=True,
        source_pieces=[{**piece, 'path': str(piece['path'])} for piece in pieces],
        captured_prefixes_verified_unchanged=True,
        github_repository='https://github.com/Sponge-BB/mppi-experience',
        github_main_sha_verified_read_only=REMOTE_SHA,
        latest_experiments_are_local_supplements=True,
        raw_sessions_copied=False, production_modified=False, github_uploaded=False)
    (OUT / '导出来源与范围.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + '\n')
    checksums = {str(path.relative_to(OUT)): digest(path.read_bytes())
                 for path in sorted(OUT.rglob('*')) if path.is_file() and path.name != 'SHA256.json'}
    (OUT / 'SHA256.json').write_text(json.dumps(checksums, ensure_ascii=False, indent=2) + '\n')
    archive = OUT.with_suffix('.zip')
    if archive.exists() and not args.refresh_generated:
        raise FileExistsError('Archive already exists')
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=5) as bundle:
        for path in sorted(OUT.rglob('*')):
            if path.is_file():
                bundle.write(path, str(Path(OUT.name) / path.relative_to(OUT)))
    with zipfile.ZipFile(archive) as bundle:
        if bundle.testzip() is not None:
            raise RuntimeError('Archive verification failed')
    print(json.dumps(dict(output=str(OUT), archive=str(archive), archive_bytes=archive.stat().st_size,
        messages=len(messages), statistics=stats, redactions=redactions,
        date_range=[messages[0]['timestamp'], messages[-1]['timestamp']], segments=len(pieces)), ensure_ascii=False))


if __name__ == '__main__':
    main()
