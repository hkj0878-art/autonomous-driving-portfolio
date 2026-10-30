"""Report correctness: source selection, drift detection and read-only dry-run."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.report.collect_metrics import ROOT, collect, extract, render
from tools.report.verify_report import verify_metrics, verify_links, dry_run_readme, verify_protected


def test_delivered_metrics_match_every_original_value_and_selector():
    assert verify_metrics() >= 70
    metrics = collect()
    source = json.loads((ROOT/'results/loc/run_stats.json').read_text())
    assert metrics['loc_rtf']['value'] == source['realtime_factor_10hz']
    assert metrics['loc_rtf']['source'] == 'results/loc/run_stats.json'
    assert metrics['loc_rtf']['display'] == format(source['realtime_factor_10hz'], '.2f')


def test_exact_csv_selection_rejects_ambiguous_and_missing_rows(tmp_path):
    (tmp_path/'table.csv').write_text('run,value\non,1.2\non,1.3\n')
    spec = dict(source='table.csv', row={'run':'on'}, column='value')
    with pytest.raises(ValueError, match='Ambiguous'):
        extract(tmp_path, spec)
    spec['row'] = {'run':'absent'}
    with pytest.raises(ValueError, match='Ambiguous'):
        extract(tmp_path, spec)


def test_text_selection_does_not_silently_choose_first_match(tmp_path):
    (tmp_path/'source.md').write_text('value 1\nvalue 2\n')
    with pytest.raises(ValueError, match='one capture'):
        extract(tmp_path, dict(source='source.md', pattern=r'value (\d+)'))


def test_source_change_changes_value_and_hash(tmp_path):
    path = tmp_path/'source.json'
    path.write_text('{"value": 1.25}')
    spec = dict(source='source.json', pointer=['value'], format='.2f')
    first = extract(tmp_path, spec)
    path.write_text('{"value": 2.25}')
    second = extract(tmp_path, spec)
    assert first['value'] != second['value']
    assert first['source_sha256'] != second['source_sha256']
    assert render('{{x}}', {'x':first}) != render('{{x}}', {'x':second})


def test_report_drift_is_rejected(tmp_path):
    (tmp_path/'report').mkdir()
    metrics = collect()
    (tmp_path/'report/metrics_headline.json').write_text(json.dumps(metrics))
    template = (ROOT/'report/REPORT.template.md').read_text()
    (tmp_path/'report/REPORT.template.md').write_text(template)
    (tmp_path/'report/REPORT.md').write_text('ATE 999 m')
    # Inject root-local source collection to isolate document-drift behavior.
    from unittest.mock import patch
    with patch('tools.report.verify_report.collect', return_value=metrics):
        with pytest.raises(ValueError, match='differ'):
            verify_metrics(tmp_path)
        (tmp_path/'report/REPORT.template.md').write_text(template+'\nUnsourced 999 m\n')
        with pytest.raises(ValueError, match='Unbound numeric'):
            verify_metrics(tmp_path)


def test_figure_links_provenance_and_readme_dry_run():
    assert verify_links()['figures'] == 9
    audit = dry_run_readme()
    assert len(audit['commands']) >= 20
    assert any('tools/eval/run_all_eval.py' in row['entrypoints'] for row in audit['commands'])
    assert verify_protected() > 0
