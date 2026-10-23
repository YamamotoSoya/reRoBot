#!/usr/bin/env python3
"""docs/ の索引を自動生成する (2026-10-23 claude)。
  docs/issue/INDEX.md    — 未解決 (issue/ 直下) と 解決・前提消失 (issue/closed/) の一覧。ステータス行は各ファイル冒頭から抽出
  docs/report/INDEX.md   — 事後報告の一覧
  docs/features/INDEX.md — 設計文書の一覧
  docs/text/README.md    — 解説書 (00_index.md を持つディレクトリ) の一覧
使い方 (ホストでもコンテナでも可): python3 docs/claude/gen_docs_index.py
索引ファイルは「自動生成」なので手で直さず、元ファイル (タイトル行・ステータス行) を直してから再生成する。
"""
import re, pathlib, datetime
ROOT = pathlib.Path(__file__).resolve().parents[2]
DOCS = ROOT / 'docs'
TODAY = datetime.date.today().isoformat()
HEAD = '<!-- 自動生成: python3 docs/claude/gen_docs_index.py ({date})。手で編集しない — 元ファイルのタイトル / ステータス行を直して再生成。 -->\n'

def title(p):
    for l in p.read_text(errors='replace').splitlines():
        if l.startswith('# '):
            return l[2:].strip()
    return p.stem

def status(p):
    txt = p.read_text(errors='replace')
    for l in txt.splitlines():
        if 'ステータス' in l and not l.lstrip().startswith('<!--'):
            s = re.sub(r'\*\*|^[-\s]+', '', l).strip()
            s = re.sub(r'^ステータス\s*(\([^)]*\))?\s*[:：]\s*', '', s)
            s = re.sub(r'\[([^\]]+)\]\([^)]*\)', r'\1', s)  # リンクは文字だけ
            return (s[:110] + '…') if len(s) > 110 else s
    return '(ステータス行なし)'

def date_of(name):
    m = re.match(r'(\d{4}-\d{2}-\d{2})', name)
    return m.group(1) if m else ''

def md_files(d):
    return sorted(p for p in d.glob('*.md') if p.name != 'INDEX.md' and not p.name.endswith('.summary.md'))

def row(p, rel_dir, with_status):
    link = f'[{p.name}]({p.relative_to(rel_dir).as_posix()})'
    summ = p.with_name(p.name[:-3] + '.summary.md')
    if summ.exists():
        link += f' / [要点版]({summ.relative_to(rel_dir).as_posix()})'
    cells = [link, date_of(p.name)]
    if with_status:
        cells.append(status(p))
    cells.append(title(p))
    return '| ' + ' | '.join(cells) + ' |'

def write(path, body):
    path.write_text(HEAD.format(date=TODAY) + body)
    print('wrote', path.relative_to(ROOT))

# issue
d = DOCS / 'issue'
open_rows = [row(p, d, True) for p in md_files(d)]
closed_rows = [row(p, d, True) for p in md_files(d / 'closed')] if (d / 'closed').exists() else []
body = f"""
# docs/issue — 調査記録の一覧

未解決・調査中は `issue/` 直下、解決・前提消失・対象廃止は `issue/closed/` に置く (2026-10-23 から)。
解決したら冒頭のステータス行を更新し `closed/` へ `git mv` → 本索引を再生成。経緯の月別まとめは [docs/monthly/](../monthly/00_index.md)。

## 未解決・調査中 ({len(open_rows)} 本)

| ファイル | 日付 | ステータス (冒頭行) | タイトル |
|---|---|---|---|
{chr(10).join(open_rows)}

## 解決・前提消失・対象廃止 ({len(closed_rows)} 本) — `closed/`

| ファイル | 日付 | ステータス (冒頭行) | タイトル |
|---|---|---|---|
{chr(10).join(closed_rows)}
"""
write(d / 'INDEX.md', body)

# report
d = DOCS / 'report'
rows = [row(p, d, False) for p in md_files(d)]
pdfs = sorted(d.glob('*.pdf'))
body = f"""
# docs/report — 解決済みバグの事後報告

`/debug-report` skill のテンプレに沿った事後報告。未解決の調査は [docs/issue/INDEX.md](../issue/INDEX.md)、設計文書は [docs/features/INDEX.md](../features/INDEX.md)。

| ファイル | 日付 | タイトル |
|---|---|---|
{chr(10).join(rows)}

PDF 版 (ゼミ資料用、同名 md と同内容): {', '.join(f'[{p.name}]({p.name})' for p in pdfs) or 'なし'}
"""
write(d / 'INDEX.md', body)

# features
d = DOCS / 'features'
rows = [row(p, d, False) for p in md_files(d)]
body = f"""
# docs/features — 追加機能の設計文書

`/feature-doc` skill のテンプレに沿った設計文書 (目的・設計の勘所・データフロー・使い方)。運用手順は [docs/manual/](../manual/00_index.md)、仕組みの教材は [docs/text/](../text/README.md)。

| ファイル | 日付 | タイトル |
|---|---|---|
{chr(10).join(rows)}
"""
write(d / 'INDEX.md', body)

# text
d = DOCS / 'text'
rows = []
for sub in sorted(x for x in d.iterdir() if x.is_dir() and (x / '00_index.md').exists()):
    n = len(list(sub.glob('*.md')))
    rows.append(f'| [{sub.name}/]({sub.name}/00_index.md) | {n} | {title(sub / "00_index.md")} |')
body = f"""
# docs/text — テーマ別解説書 (書籍形式)

1 テーマ = 1 ディレクトリ、`00_index.md` + 章分割。作成・更新は `/textbook` skill。読む順の目安: 仕組みを知りたいテーマの 00_index → 「reRoBot での構成」章 → 事例解剖 → チェックリスト。

| 解説書 | ファイル数 | タイトル |
|---|---|---|
{chr(10).join(rows)}
"""
write(d / 'README.md', body)
