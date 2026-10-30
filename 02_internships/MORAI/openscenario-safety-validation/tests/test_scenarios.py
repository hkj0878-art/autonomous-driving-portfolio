from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree as ET
import pytest
from tools.validate_scenario import validate_scenario,validate_distribution

ROOT=Path(__file__).resolve().parents[1]
SCENE=ROOT/'scenarios/ego_lane_change_then_stop.xosc'
DIST=ROOT/'scenarios/ego_speed_distribution.xml'


def _mutate(tmp_path, source, modifier):
    tree=ET.parse(source)
    modifier(tree.getroot())
    dest=tmp_path/source.name
    tree.write(dest, encoding='utf-8',xml_declaration=True)
    return dest


def test_xml_well_formed():
    assert ET.parse(SCENE).getroot().tag=='OpenSCENARIO'
    assert ET.parse(DIST).getroot().tag=='OpenSCENARIO'


def test_event_chain():
    warnings=validate_scenario(SCENE)
    assert len(warnings)==1 and 'map reference not found' in warnings[0]


def test_distribution_matches_scenario():
    assert validate_distribution(DIST,SCENE)==[]


def test_event_trigger_after_2_seconds():
    tree=ET.parse(SCENE)
    event=next(e for e in tree.findall('.//Event') if e.get('name')=='Ego_lanechange')
    time=event.find('.//SimulationTimeCondition')
    assert time.get('rule')=='greaterThan'
    assert float(time.get('value'))==2.0


def test_speed_kph_conversion():
    d=ET.parse(DIST)
    values=[float(el.get('value')) for el in d.findall('.//DistributionSet/Element')]
    assert len(values)==17
    assert values[0]*3.6==pytest.approx(20,abs=1e-6)
    assert values[-1]*3.6==pytest.approx(100,abs=1e-6)


def test_reject_invalid_rule(tmp_path):
    def change(root):root.find('.//SimulationTimeCondition').set('rule','greaterOrEqual')
    with pytest.raises(ValueError,match='comparison rule'):
        validate_scenario(_mutate(tmp_path,SCENE,change))


def test_reject_undefined_speed(tmp_path):
    def change(root):root.find('.//ParameterDeclaration[@name="egoSpeed"]').set('name','other')
    with pytest.raises(ValueError,match='undeclared referenced parameter'):
        validate_scenario(_mutate(tmp_path,SCENE,change))


def test_reject_wrong_event_ref(tmp_path):
    def change(root):root.find('.//StoryboardElementStateCondition').set('storyboardElementRef','missing')
    with pytest.raises(ValueError,match='stop must follow'):
        validate_scenario(_mutate(tmp_path,SCENE,change))


def test_reject_missing_ego(tmp_path):
    def change(root):root.find('.//ScenarioObject').set('name','NPC')
    with pytest.raises(ValueError,match='Ego entity'):
        validate_scenario(_mutate(tmp_path,SCENE,change))


def test_reject_invalid_speed_range(tmp_path):
    def change(root):root.find('.//DistributionSet/Element').set('value','20')
    with pytest.raises(ValueError,match='speed sweep'):
        validate_distribution(_mutate(tmp_path,DIST,change),SCENE)


def test_reject_dtd(tmp_path):
    target=tmp_path/'unsafe.xosc'
    target.write_text('<!DOCTYPE foo [<!ENTITY x "value">]><OpenSCENARIO/>')
    with pytest.raises(ValueError,match='DTD'):
        validate_scenario(target)


def test_missing_map_reported_not_ignored():
    warnings=validate_scenario(SCENE)
    assert any('MGeo' in item for item in warnings)
