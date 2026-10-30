"""Read-only report, copied-image and README entry-point audit."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.report.collect_metrics import ROOT, collect, render


def verify_metrics(root=ROOT):
    expected = collect(root)
    if json.loads((root/'report/metrics_headline.json').read_text()) != expected:
        raise ValueError('Metric value or source provenance mismatch')
    template = (root/'report/REPORT.template.md').read_text()
    # Numeric facts must be placeholders. Permit only notation/identifiers in prose.
    prose = re.sub(r'```[\s\S]*?```|`[^`]*`|\[[^\]]*\]\([^)]*\)|\[\^[^\]]+\]', '', template)
    prose = re.sub(r'\{\{\w+\}\}|SE3|2D|3D|2-DoF|3-DoF|v1|v2|v3|p95|Nav2|rev2|TASK-05', '', prose)
    if re.search(r'\d', prose):
        raise ValueError('Unbound numeric fact in report template')
    if (root/'report/REPORT.md').read_text() != render(template, expected):
        raise ValueError('Report text/numbers differ from source-backed rendering')
    return len(expected)


def verify_links(root=ROOT):
    count = 0
    for rel in ('report/REPORT.md', 'REPRODUCE.md'):
        doc = root/rel
        for target in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)', doc.read_text()):
            if target.startswith(('http:', 'https:', '#')):
                continue
            if not (doc.parent/target.split('#')[0]).exists():
                raise ValueError(f'Broken link in {rel}: {target}')
            count += 1
    doc = (root/'report/REPORT.md').read_text()
    refs = set(re.findall(r'\[\^([^\]]+)\](?!:)', re.sub(r'`[^`]*`', '', doc)))
    definitions = set(re.findall(r'^\[\^([^\]]+)\]:', doc, re.M))
    if refs != definitions:
        raise ValueError('Missing or unused source footnote')
    figures = json.loads((root/'report/figures.json').read_text())
    image_links = re.findall(r'!\[[^\]]*\]\(([^)]+)\)', doc)
    if not 0 < len(image_links) <= 10 or len(figures) != len(image_links):
        raise ValueError('Figure count or manifest mismatch')
    for target in image_links:
        rel = str(Path('report')/target)
        spec = figures[rel]
        for path in (root/rel, root/spec['source']):
            if hashlib.sha256(path.read_bytes()).hexdigest() != spec['sha256']:
                raise ValueError(f'Figure copy/source mismatch: {path}')
        if f"원본: `{spec['source']}`" not in doc:
            raise ValueError('Missing figure provenance caption')
    return dict(links=count, figures=len(figures))


def dry_run_readme(root=ROOT):
    """Parse shell syntax and resolve every local command entry point; execute none."""
    text = (root/'REPRODUCE.md').read_text()
    blocks = re.findall(r'```bash\n([\s\S]*?)```', text)
    checked = []
    for block in blocks:
        subprocess.run(['bash', '-n'], input=block, text=True, check=True, capture_output=True)
        for raw in block.splitlines():
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            paths = re.findall(r'(?:source|bash)\s+([\w./-]+)', line)
            paths += re.findall(r'\.venv/bin/python\s+((?:tools/)[\w/.-]+\.py)', line)
            modules = re.findall(r'\.venv/bin/python\s+-m\s+(tools\.[\w.]+)', line)
            paths += [m.replace('.', '/')+'.py' for m in modules]
            if '.venv/bin/python' in line:
                paths.append('.venv/bin/python')
            for target in paths:
                if not (root/target).is_file():
                    raise ValueError(f'Missing README entry point: {target}')
            checked.append(dict(command=line, entrypoints=paths))
    for target in ('data/synced_bag/metadata.yaml', 'config/params_base.yaml',
                   'config/params_loop_off.yaml', 'third_party/gtsam/CMakeLists.txt',
                   'ws/src/LIO-SAM/CMakeLists.txt', 'report/requirements.txt'):
        if not (root/target).is_file():
            raise ValueError(f'Missing prerequisite: {target}')
    if not checked:
        raise ValueError('No commands checked')
    return dict(mode='syntax + file/script/module existence only; no execution', commands=checked)


def verify_protected(root=ROOT):
    before = json.loads((root/'logs/task07_protected_before.json').read_text())
    after = {str(p.relative_to(root)): [p.stat().st_size, p.stat().st_mtime_ns]
             for d in ('data', 'results') for p in (root/d).rglob('*') if p.is_file()}
    if before != after:
        raise ValueError('Protected tree file set, size or mtime changed')
    return len(after)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dry-run-readme', action='store_true')
    args = p.parse_args()
    result = dict(metrics=verify_metrics(), **verify_links(), protected_files=verify_protected())
    if args.dry_run_readme:
        result['readme'] = dry_run_readme()
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
