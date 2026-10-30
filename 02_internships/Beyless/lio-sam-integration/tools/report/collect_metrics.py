"""Copy existing result values with exact provenance; never recompute metrics."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
SPEC = Path(__file__).with_name('sources.json')


def extract(root, spec):
    path = root / spec['source']
    if 'pointer' in spec:
        value = json.loads(path.read_text())
        for part in spec['pointer']:
            value = value[part]
    elif 'row' in spec:
        with path.open(newline='') as stream:
            rows = [row for row in csv.DictReader(stream)
                    if all(row[k] == v for k, v in spec['row'].items())]
        if len(rows) != 1:
            raise ValueError(f'Ambiguous CSV selector: {spec}')
        value = rows[0][spec['column']]
    else:
        matches = re.findall(spec['pattern'], path.read_text(), re.MULTILINE)
        if len(matches) != 1 or not isinstance(matches[0], str):
            raise ValueError(f'Expected one capture: {spec}')
        value = matches[0]
    display = str(value)
    if 'format' in spec:
        display = format(float(value), spec['format'])
    return dict(value=value, display=display, source=spec['source'],
                selector={k: v for k, v in spec.items() if k != 'source'},
                source_sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def collect(root=ROOT, specs=None):
    specs = json.loads(SPEC.read_text()) if specs is None else specs
    return {key: extract(root, spec) for key, spec in specs.items()}


def render(template, metrics):
    used = []
    def replace(match):
        key = match[1]
        if key not in used:
            used.append(key)
        return metrics[key]['display'] + f'[^m_{key}]'
    body = re.sub(r'\{\{(\w+)\}\}', replace, template)
    notes = [f"[^m_{key}]: [{metrics[key]['source']}](../{metrics[key]['source']}); "
             f"selector: `{json.dumps(metrics[key]['selector'], ensure_ascii=False)}`."
             for key in used]
    return body.rstrip() + '\n\n' + '\n'.join(notes) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    metrics = collect()
    destination = ROOT / 'report/metrics_headline.json'
    report = ROOT / 'report/REPORT.md'
    rendered = render((ROOT / 'report/REPORT.template.md').read_text(), metrics)
    if args.check:
        if json.loads(destination.read_text()) != metrics:
            raise ValueError('Metrics/provenance differs from source')
        if report.read_text() != rendered:
            raise ValueError('Report differs from verified source rendering')
    else:
        destination.write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + '\n')
        report.write_text(rendered)
    print(f'{len(metrics)} source values: PASS')


if __name__ == '__main__':
    main()
