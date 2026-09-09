from copy import deepcopy
from pathlib import Path
import json
import numpy as np
import pytest
from tools.lib.so101_mesh_pinch import (
    ActualMeshPinchGate, MeshPinchError, clipped_minimum_x, load_closure_guard,
)

ROOT = Path(__file__).resolve().parents[1]


def contacts():
    f = dict(shape='/Robot/left_gripper_link/TowelFixedJawCollider', particle=0,
             surface_world_m=[0,0,0], surface_local_m=[-.0057,0,0],
             particle_world_m=[.0002,0,0], normal_world=[1.,0,0], normal_local=[1.,0,0],
             penetration_m=.0008, signed_distance_m=.0002, radius_m=.001)
    m = dict(shape='/Robot/left_moving_jaw_link/moving_jaw_so101_v1', particle=1,
             surface_world_m=[.001,0,0], surface_local_m=[0,0,0],
             particle_world_m=[.0008,.001,0], normal_world=[-1.,0,0], normal_local=[-1.,0,0],
             penetration_m=.0008, signed_distance_m=.0002, radius_m=.001)
    return [f,m]


def test_triangle_crossing_prism_without_inside_vertices_is_detected():
    equations = [[1,0,-1],[-1,0,-1],[0,1,-1],[0,-1,-1]]
    triangle = [[[.2,-3,-3],[.2,3,-3],[.2,0,3]]]
    assert clipped_minimum_x(triangle, equations) == pytest.approx(.2)
    assert clipped_minimum_x(np.array(triangle)+[0,10,0], equations) == float('inf')


def test_actual_stl_guard_blocks_old_closure_and_matches_both_sides():
    pad=json.loads((ROOT/'artifacts/bimanual/planning/so101_surface_matched_pad_20260906/geometry.json').read_text())
    for side, old in [('left',-.064985),('right',-.006694)]:
        guard,_ = load_closure_guard(pad, ROOT/'artifacts/bimanual/preview/so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf', ROOT, side)
        stop=guard.derive_stop(old,.4)
        assert guard.clearance(old)<0
        assert stop['minimum_model_angle_rad']==pytest.approx(.0069799024,abs=1e-8)
        assert guard.clearance(stop['minimum_model_angle_rad'])==pytest.approx(.00025,abs=1e-9)
        assert guard.clearance(stop['minimum_model_angle_rad']-.001)<.00025


def test_opposed_actual_contacts_use_actual_triangle_support():
    gate=ActualMeshPinchGate([[0,1,2],[2,3,4]])
    result=gate.evaluate(contacts(),'left',.006)
    assert result['passed']
    assert result['finite_element_support_vertex_indices']==[0,1,2]
    for anchor, contact in zip(result['anchors'], contacts(), strict=True):
        weights = np.asarray(anchor['barycentric_weights'])
        assert weights.sum() == 1
        assert anchor['triangle_indices'][int(weights.argmax())] == contact['particle']
    assert not gate.evaluate([], 'left', .006)['passed']  # Lost current contacts cannot reuse prior success.


def test_upright_pinch_requires_material_span_across_closing_direction():
    gate = ActualMeshPinchGate([[0, 1, 3]], grid_side=3, required_material_axis="column")
    assert gate.evaluate(contacts(), 'left', .006)['passed']
    lateral = deepcopy(contacts())
    lateral[1]['particle'] = 3
    assert not gate.evaluate(lateral, 'left', .006)['passed']
    assert ActualMeshPinchGate([[0, 1, 3]]).evaluate(lateral, 'left', .006)['passed']


def test_free_contact_hold_allows_one_particle_between_opposing_jaws():
    pair = deepcopy(contacts())
    pair[1]['particle'] = 0
    pair[1]['particle_world_m'] = pair[0]['particle_world_m']
    gate = ActualMeshPinchGate([[0, 1, 2]])
    assert not gate.evaluate(pair, 'left', .006)['passed']
    result = gate.evaluate(pair, 'left', .006, allow_same_particle=True)
    assert result['passed'] and result['unique_contact_particle_count'] == 1
    assert result['selected_distinct_particles'] == [0]
    pair[1]['normal_world'] = [1., 0, 0]
    assert not gate.evaluate(pair, 'left', .006, allow_same_particle=True)['passed']
    pair[1]['normal_world'] = [-1., 0, 0]
    pair[1]['penetration_m'] = 0
    assert not gate.evaluate(pair, 'left', .006, allow_same_particle=True)['passed']


def test_native_outer_perimeter_still_requires_axial_force_and_outer_plane():
    gate = ActualMeshPinchGate([[0, 1, 2]])
    pair = contacts()
    for angle, expected in [(3, True), (16, True), (59, True), (61, False), (90, False)]:
        a = np.deg2rad(angle)
        pair[0]['normal_world'] = pair[0]['normal_local'] = [np.cos(a), np.sin(a), 0.]
        assert gate.evaluate(pair, 'left', .006,
            allow_outer_face_boundary=True)['passed'] is expected
        assert not gate.evaluate(pair, 'left', .006)['passed']
    pair[0]['normal_world'] = pair[0]['normal_local'] = [1., 0., 0.]
    pair[0]['surface_local_m'][0] = -.006  # side rim is not the outer plane
    assert not gate.evaluate(pair, 'left', .006, allow_outer_face_boundary=True)['passed']


def test_actual_friction_cones_qualify_curved_contacts_without_fixed_normal_angle():
    gate = ActualMeshPinchGate([[0, 1, 2]])
    pair = contacts()
    a = np.deg2rad(35.)
    pair[0]['normal_world'] = pair[0]['normal_local'] = [np.cos(a), np.sin(a), 0.]
    pair[1]['normal_world'] = [-np.cos(a), np.sin(a), 0.]
    for c in pair: c['effective_friction_coefficient'] = .8
    assert not gate.evaluate(pair, 'left', .006, allow_outer_face_boundary=True)['passed']
    assert gate.evaluate(pair, 'left', .006, allow_outer_face_boundary=True, friction_cone_check=True)['passed']
    pair[1]['effective_friction_coefficient'] = .4
    assert not gate.evaluate(pair, 'left', .006, allow_outer_face_boundary=True, friction_cone_check=True)['passed']
    pair[1]['effective_friction_coefficient'] = float('nan')
    with pytest.raises(MeshPinchError):
        gate.evaluate(pair, 'left', .006, allow_outer_face_boundary=True, friction_cone_check=True)


@pytest.mark.parametrize('case', ['same_particle','different_triangle','same_normal','back_face','inside_rigid','speculative','far_pair','wrong_side','crossed_surfaces'])
def test_false_pinch_rejected(case):
    pair=deepcopy(contacts())
    if case=='same_particle': pair[1]['particle']=0
    if case=='different_triangle': pair[1]['particle']=3
    if case=='same_normal': pair[1]['normal_world']=[1.,0,0]
    if case=='back_face': pair[0]['surface_local_m'][0]=-.0079
    if case=='inside_rigid': pair[0]['signed_distance_m']=-.0005
    if case=='speculative': pair[0]['penetration_m']=0
    if case=='far_pair': pair[1]['particle_world_m'][1]=.1
    if case=='wrong_side': pair[0]['shape']=pair[0]['shape'].replace('left','right')
    if case=='crossed_surfaces': pair[1]['surface_world_m'][0]=-.001
    assert not ActualMeshPinchGate([[0,1,2],[2,3,4]]).evaluate(pair,'left',.006)['passed']


def test_persistent_closure_resumes_only_lost_side_and_requires_fresh_hold():
    from tools.lib.so101_mesh_pinch import PersistentMeshClosure
    c=PersistentMeshClosure({'left':1.,'right':1.},{'left':0.,'right':0.},10,3)
    yes={'passed':True,'finite_element_support_vertex_indices':[0,1,2]}
    no={'passed':False}
    assert not c.observe({'left':yes,'right':yes})
    assert not c.observe({'left':no,'right':yes})
    assert c.targets == pytest.approx({'left':.9,'right':1.})
    assert c.losses['left']==1
    assert not c.observe({'left':yes,'right':yes})
    assert not c.observe({'left':yes,'right':yes})
    assert c.observe({'left':yes,'right':yes})
    assert not c.observe({'left':no,'right':yes})
    for _ in range(50):c.observe({'left':no,'right':yes})
    assert c.targets['left']==0.
    assert c.targets['right']==1.


def test_changing_contact_triangle_restarts_persistence_window():
    from tools.lib.so101_mesh_pinch import PersistentMeshClosure
    c=PersistentMeshClosure({'left':1.},{'left':0.},10,2)
    assert not c.observe({'left':{'passed':True,'finite_element_support_vertex_indices':[0,1,2]}})
    assert not c.observe({'left':{'passed':True,'finite_element_support_vertex_indices':[1,2,3]}})
    assert c.observe({'left':{'passed':True,'finite_element_support_vertex_indices':[3,2,1]}})


def test_required_triangle_checks_original_support_instead_of_selector_order():
    gate=ActualMeshPinchGate([[0,1,2],[0,1,3]])
    assert gate.evaluate(contacts(),'left',.006)['support_triangle_index']==0
    result=gate.evaluate(contacts(),'left',.006,required_triangle_index=1)
    assert result['passed'] and result['support_triangle_index']==1
    assert result['finite_element_support_vertex_indices']==[0,1,3]
    pair=contacts();pair[1]['particle']=2
    assert not gate.evaluate(pair,'left',.006,required_triangle_index=1)['passed']


def test_fixed_pad_last_release_uses_roles_not_particle_number_or_triangle_order():
    from tools.lib.so101_mesh_pinch import fixed_pad_last_release_order
    from itertools import permutations
    for support in permutations([42, 3, 17]):
        evidence = {'passed':True, 'finite_element_support_vertex_indices':support,
                    'selected_distinct_particles':[42, 3]}
        assert fixed_pad_last_release_order(evidence) == [17, 3, 42]
        evidence['selected_distinct_particles'] = [3, 42]
        assert fixed_pad_last_release_order(evidence) == [17, 42, 3]
    with pytest.raises(MeshPinchError):
        fixed_pad_last_release_order({'passed':False})
