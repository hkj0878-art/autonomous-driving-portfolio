"""Check local OpenSCENARIO examples for common reference and trigger mistakes.

This is not an ASAM schema checker or a MORAI ScenarioRunner execution test.
"""
from __future__ import annotations
import argparse
from pathlib import Path
from xml.etree import ElementTree as ET
from math import isfinite

# OpenSCENARIO XML 1.0 enumeration (ASAM); not an exhaustive implementation.
RULES = {'equalTo', 'greaterThan', 'lessThan'}
EDGES = {'none', 'rising', 'falling', 'risingOrFalling'}


def _read(path: Path) -> ET.Element:
    raw = path.read_bytes()
    if b'<!DOCTYPE' in raw.upper() or b'<!ENTITY' in raw.upper():
        raise ValueError('DTD and entity declarations are not allowed')
    return ET.fromstring(raw)


def _positive(value: str, what: str, *, zero: bool = True) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError) as ex:
        raise ValueError(f'{what}: not a number') from ex
    if not isfinite(v) or (v < 0 if zero else v <= 0):
        raise ValueError(f'{what}: invalid nonnegative value')
    return v


def validate_scenario(path: Path) -> list[str]:
    root = _read(path)
    if root.tag != 'OpenSCENARIO':
        raise ValueError('wrong root element')
    header = root.find('./FileHeader')
    if header is None or header.get('revMajor') != '1':
        raise ValueError('missing or unsupported FileHeader')
    if root.find('./RoadNetwork/LogicFile') is None:
        raise ValueError('RoadNetwork/LogicFile missing')
    declarations = root.findall('./ParameterDeclarations/ParameterDeclaration')
    params = [p.get('name') for p in declarations]
    if len(params) != len(set(params)):
        raise ValueError('duplicate parameter declaration')
    entities = {e.get('name') for e in root.findall('./Entities/ScenarioObject')}
    if 'Ego' not in entities:
        raise ValueError('Ego entity not declared')
    for ref in root.findall('.//EntityRef'):
        if ref.get('entityRef') not in entities:
            raise ValueError(f'entity reference missing: {ref.get("entityRef")}')
    events = root.findall('.//Event')
    names = [e.get('name') for e in events]
    if len(names) != len(set(names)):
        raise ValueError('duplicate event')
    if not {'Ego_lanechange', 'Ego_stop'}.issubset(names):
        raise ValueError('lane change and stop events required')
    change = next(e for e in events if e.get('name') == 'Ego_lanechange')
    stop = next(e for e in events if e.get('name') == 'Ego_stop')
    if change.find('.//LaneChangeAction') is None or stop.find('.//AbsoluteTargetSpeed[@value="0.0"]') is None:
        raise ValueError('lane-change or stop action missing')
    if change.find('.//SimulationTimeCondition') is None:
        raise ValueError('time trigger missing')
    state = stop.find('./StartTrigger//StoryboardElementStateCondition')
    if state is None or state.get('storyboardElementRef') != 'Ego_lanechange' or state.get('state') != 'completeState' or state.get('storyboardElementType') != 'event':
        raise ValueError('stop must follow lane-change completion')
    for condition in root.findall('.//Condition'):
        if condition.get('conditionEdge') not in EDGES:
            raise ValueError('invalid conditionEdge')
        _positive(condition.get('delay'), 'delay')
    for c in root.findall('.//SimulationTimeCondition') + root.findall('.//ParameterCondition'):
        if c.get('rule') not in RULES:
            raise ValueError('invalid comparison rule')
    for p in root.findall('.//ParameterCondition'):
        if p.get('parameterRef') not in params:
            raise ValueError('undefined condition parameter')
    for tag in root.iter():
        for value in tag.attrib.values():
            if value.startswith('$') and value[1:] not in params:
                raise ValueError(f'undeclared referenced parameter: {value}')
    if 'egoSpeed' not in params:
        raise ValueError('egoSpeed not declared')
    warnings = []
    network_path = root.find('./RoadNetwork/LogicFile').get('filepath')
    if not (path.parent / network_path).exists():
        warnings.append(f'map reference not found locally: {network_path}')
    return warnings


def validate_distribution(path: Path, scenario: Path) -> list[str]:
    root = _read(path)
    if root.tag != 'OpenSCENARIO':
        raise ValueError('wrong distribution root')
    ref = root.find('./ParameterValueDistribution/ScenarioFile')
    if ref is None or ref.get('filepath') != scenario.name:
        raise ValueError('ScenarioFile must point to scenario')
    available = {p.get('name') for p in _read(scenario).findall('./ParameterDeclarations/ParameterDeclaration')}
    distributions = root.findall('./ParameterValueDistribution/Deterministic/DeterministicSingleParameterDistribution')
    if len(distributions) != 1 or distributions[0].get('parameterName') != 'egoSpeed':
        raise ValueError('egoSpeed distribution required')
    if distributions[0].get('parameterName') not in available:
        raise ValueError('distribution parameter is undefined')
    values = [_positive(p.get('value'), 'speed', zero=False) for p in distributions[0].findall('./DistributionSet/Element')]
    expected = [i/3.6 for i in range(20,101,5)]
    if len(values) != len(expected) or any(abs(v-e)>1e-6 for v,e in zip(values,expected)):
        raise ValueError('speed sweep should represent 20-100 km/h in m/s, step 5 km/h')
    return []


def validate_xsd(xml: Path, schema: Path) -> None:
    """Full XSD check with user-supplied appropriate-version schema (optional)."""
    from lxml import etree
    xsd = etree.XMLSchema(etree.parse(str(schema)))
    doc = etree.parse(str(xml))
    xsd.assertValid(doc)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', type=Path, default=Path('scenarios/ego_lane_change_then_stop.xosc'))
    parser.add_argument('--distribution', type=Path, default=Path('scenarios/ego_speed_distribution.xml'))
    parser.add_argument('--xsd',type=Path,help='optional OpenSCENARIO XML schema matching simulator version')
    args = parser.parse_args()
    warnings = validate_scenario(args.scenario)
    validate_distribution(args.distribution, args.scenario)
    if args.xsd:
        validate_xsd(args.scenario,args.xsd)
        validate_xsd(args.distribution,args.xsd)
        print('XSD: PASS')
    else:
        print('XSD: NOT RUN (pass --xsd to check)')
    print('XML + local reference checks: PASS')
    for warning in warnings:
        print('NOTE:',warning)


if __name__ == '__main__':
    main()
