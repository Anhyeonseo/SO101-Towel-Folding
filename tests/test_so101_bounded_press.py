import json
from pathlib import Path
import pytest
from tools.lib.so101_bounded_press import JawDeflectionStop


def make_stop():
    return JawDeflectionStop(threshold_rad=.005, dwell_s=.05, maximum_rad=.08)


def test_brief_transient_is_rejected_and_sustained_encoder_deflection_stops():
    stop = make_stop()
    assert not stop.observe(0, .18, .17)['stop_descent']
    assert not stop.observe(.02, .18, .18)['stop_descent']
    assert not stop.observe(.03, .18, .17)['stop_descent']
    assert stop.observe(.08, .18, .17)['stop_descent']


def test_opening_or_invalid_encoders_cannot_authorize_descent_stop():
    stop = make_stop()
    assert not stop.observe(0, .18, .19)['stop_descent']
    assert not stop.observe(.1, .18, .19)['stop_descent']
    with pytest.raises(ValueError): stop.observe(.2, .18, float('nan'))
    with pytest.raises(RuntimeError): make_stop().observe(0, .18, 0)


def test_recorded_free_motion_does_not_trigger_contact_stop():
    root = Path(__file__).resolve().parents[1]
    path = root/'artifacts/bimanual/planning/r2_contact_foundation_20260908/position_one_way/result.json'
    data = json.loads(path.read_text())
    stop = make_stop()
    for sample in data['drive_trace']:
        if sample['phase'] == 'u_pinch_contact':
            assert not stop.observe(sample['time_s'], sample['target_rad'][5], sample['actual_rad'][5])['stop_descent']
