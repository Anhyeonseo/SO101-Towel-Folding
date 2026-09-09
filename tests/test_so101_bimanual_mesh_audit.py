"""Regression checks for collision-screen geometry and conservative bounds."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages', '/usr/lib/python3/dist-packages'])
coal = pytest.importorskip('coal')
trimesh = pytest.importorskip('trimesh')
from tools.lib.so101_bimanual_mesh_audit import BimanualMeshAudit, mesh_model, aabb_distance

ROOT = Path(__file__).resolve().parents[1]


def test_mesh_distances_and_solid_table_containment():
    mesh = trimesh.creation.box([.01, .01, .01])
    obj = coal.CollisionObject(mesh_model(mesh.vertices, mesh.faces))
    solid = coal.CollisionObject(coal.Box(.02, .02, .02))
    req = coal.DistanceRequest()
    obj.setTransform(coal.Transform3s(np.eye(3), np.array([.025, 0., 0.])))
    assert coal.distance(obj, solid, req, coal.DistanceResult()) == pytest.approx(.01)
    obj.setTransform(coal.Transform3s(np.eye(3), np.zeros(3)))
    assert coal.distance(obj, solid, req, coal.DistanceResult()) <= 0


@pytest.fixture(scope='module')
def audit():
    pad = json.loads((ROOT / 'artifacts/bimanual/planning/so101_surface_matched_pad_20260906/geometry.json').read_text())
    return BimanualMeshAudit(ROOT,
        ROOT / 'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf',
        pad, {'size_xyz_m': [.37729576435909806, .37151273763179776, .02],
              'pose_xyz_m': [.32907949571699413, -.12131763166189194, -.015]})


def test_every_cross_arm_pair_including_mounts_is_checked(audit):
    pairs = {(i, j) for i, j, _, _ in audit.pairs}
    for i, a in enumerate(audit.entries):
        for j, b in enumerate(audit.entries[i+1:], i+1):
            if {a['side'], b['side']} == {'left', 'right'}:
                assert (i, j) in pairs
    assert len(audit.mounts) == 2


def test_pad_stop_distance_is_invariant_to_arm_pose(audit):
    from tools.lib.so101_mesh_pinch import load_closure_guard
    pad = json.loads((ROOT / 'artifacts/bimanual/planning/so101_surface_matched_pad_20260906/geometry.json').read_text())
    guard, _ = load_closure_guard(pad, ROOT / 'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf', ROOT, 'left')
    stop = guard.derive_stop(-.023567, .18658800423145294)['minimum_model_angle_rad']
    rng = np.random.default_rng(10)
    for _ in range(3):
        q = rng.uniform(-.5, .5, 12); q[5] = stop
        audit.state(q)
        jaw = next(e for e in audit.entries if e['link'] == 'left_moving_jaw_link')
        pad_entry = next(e for e in audit.entries if e['name'] == 'left:rubber_pad')
        distance = coal.distance(jaw['obj'], pad_entry['obj'], coal.DistanceRequest(), coal.DistanceResult())
        assert distance == pytest.approx(.00025, abs=1e-10)


def test_motion_bound_dominates_pair_vertex_motion(audit):
    rng = np.random.default_rng(11)
    qa = rng.uniform(-.4, .4, 12); qb = qa + rng.uniform(-.02, .02, 12)
    positions = []
    for q in (qa, qb):
        audit.state(q)
        positions.append([np.asarray(e['obj'].getRotation()) @ e['corners'].T + np.asarray(e['obj'].getTranslation())[:, None] for e in audit.entries])
    displacements = [np.linalg.norm(b-a, axis=0).max() for a, b in zip(*positions)]
    measured = max(displacements[i] + displacements[j] for i, j, _, _ in audit.pairs)
    assert measured <= audit.motion_bound(qa, qb) + 1e-12


def test_invalid_state_rejected_and_aabb_lower_bound(audit):
    with pytest.raises(ValueError):
        audit.state([0.] * 11)
    with pytest.raises(ValueError):
        audit.state([float('nan')] * 12)
    assert aabb_distance((np.zeros(3), np.ones(3)), (np.array([2., 2., 0.]), np.array([3., 3., 1.]))) == pytest.approx(np.sqrt(2))
