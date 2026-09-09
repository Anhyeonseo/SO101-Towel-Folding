import json

import pytest

from tools.run.audit_towel_s2_corner_contact import angle_deg, literal_assignment, load_locked_source


def test_normal_comparison_preserves_direction_and_normalizes_length():
    assert angle_deg([2, 0, 0], [-3, 0, 0]) == pytest.approx(180)
    assert angle_deg([2, 0, 0], [3, 0, 0]) == pytest.approx(0)
    with pytest.raises(ValueError):
        angle_deg([0, 0, 0], [1, 0, 0])


def test_authored_geometry_is_read_without_running_simulator(tmp_path):
    source = tmp_path / "runner.py"
    source.write_text('raise RuntimeError("must not execute")\nNORMAL = {"right": (0, 0, 1)}\n')
    assert literal_assignment(source, "NORMAL") == {"right": (0, 0, 1)}
    source.write_text("NORMAL = (1, 0, 0)\nNORMAL = (0, 0, 1)\n")
    with pytest.raises(ValueError, match="one literal"):
        literal_assignment(source, "NORMAL")


def test_stale_geometry_source_is_rejected(tmp_path):
    source = tmp_path / "raw.json"
    source.write_text(json.dumps({"motion_authorized": False}))
    with pytest.raises(ValueError, match="hash mismatch"):
        load_locked_source({"path": str(source), "sha256": "0" * 64})
