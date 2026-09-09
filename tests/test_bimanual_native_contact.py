import numpy as np
import pytest
from tools.lib.so101_bimanual_native_contact import arm_contacts, both_layers_lifted


def test_other_arm_cannot_supply_contact_and_geometry_is_unchanged():
    source = [{'shape': '/World/left_gripper_link/TowelFixedJawCollider', 'particle': 1,
               'normal_world': [1., 0., 0.]},
              {'shape': '/World/right_gripper_link/right_moving_jaw_link/moving_jaw_so101',
               'particle': 2, 'normal_world': [0., 1., 0.]}]
    selected = arm_contacts(source, 'right')
    assert len(selected) == 1 and selected[0]['particle'] == 2
    assert selected[0]['normal_world'] == [0., 1., 0.]
    assert '/left_moving_jaw_link/' in selected[0]['shape']
    assert '/right_moving_jaw_link/' in source[1]['shape']
    assert arm_contacts(source, 'left')[0]['particle'] == 1


def test_one_arm_or_one_layer_cannot_hide_failed_lift():
    zero = np.zeros((4, 3)); lifted = zero.copy(); lifted[:, 2] = .01
    material = {'left': {0: [0], 1: [1]}, 'right': {0: [2], 1: [3]}}
    capture = {'left': zero.copy(), 'right': zero.copy()}
    assert both_layers_lifted(capture, lifted, material)['passed']
    lifted[3, 2] = .004
    assert not both_layers_lifted(capture, lifted, material)['passed']
    with pytest.raises(ValueError):
        both_layers_lifted(capture, lifted, {'left': material['left']})


def test_lift_is_measured_from_each_arms_own_capture_time():
    a = np.zeros((4, 3)); b = a.copy(); b[:, 2] = .008
    current = b.copy(); current[:, 2] = .012
    material = {'left': {0: [0], 1: [1]}, 'right': {0: [2], 1: [3]}}
    result = both_layers_lifted({'left': a, 'right': b}, current, material)
    assert not result['passed']
    assert result['layer_lift_m']['right']['0'] == pytest.approx(.004)


def test_frozen_two_point_patch_keeps_roles_and_rejects_replacements():
    from copy import deepcopy
    from tools.lib.so101_mesh_pinch import ActualMeshPinchGate
    from tools.lib.so101_bimanual_native_contact import freeze_layer_pairs, retained_layer_pairs, frozen_pair_indices
    records = []
    for first in [0, 3]:
        records += [dict(shape='/Robot/left_gripper_link/TowelFixedJawCollider',
            particle=first, surface_world_m=[0,0,0], surface_local_m=[-.0057,0,0],
            particle_world_m=[.0002,0,0], normal_world=[1.,0,0], normal_local=[1.,0,0],
            penetration_m=.0008, signed_distance_m=.0002, radius_m=.001,
            effective_friction_coefficient=.8),
            dict(shape='/Robot/left_moving_jaw_link/moving_jaw_so101_v1',
            particle=first+1, surface_world_m=[.001,0,0], surface_local_m=[0,0,0],
            particle_world_m=[.0008,.0001,0], normal_world=[-1.,0,0], normal_local=[-1.,0,0],
            penetration_m=.0008, signed_distance_m=.0002, radius_m=.001,
            effective_friction_coefficient=.8)]
    gate = ActualMeshPinchGate([[0,1,2],[3,4,5]])
    evidence = {'passed':True, 'layers':{str(h):gate.evaluate(records[2*h:2*h+2],
        'left', .02, allow_same_particle=True, friction_cone_check=True) for h in (0,1)}}
    frozen = freeze_layer_pairs(evidence)
    assert frozen_pair_indices(frozen) == {0:[0,1], 1:[3,4]}
    assert retained_layer_pairs(records,gate,frozen)['passed']
    replacement = deepcopy(records); replacement[1]['particle'] = 2
    assert not retained_layer_pairs(replacement,gate,frozen)['passed']
    missing = records[:3]
    assert not retained_layer_pairs(missing,gate,frozen)['passed']
    speculative = deepcopy(records); speculative[3]['penetration_m'] = 0
    assert not retained_layer_pairs(speculative,gate,frozen)['passed']
    oblique = deepcopy(records)
    for i in [1, 3]:
        oblique[i]['surface_world_m'][1] = .002
        oblique[i]['particle_world_m'][1] = .002
    assert not retained_layer_pairs(oblique, gate, frozen)['passed']
    assert retained_layer_pairs(oblique, gate, frozen, friction_cone_check=False)['passed']
    assert not retained_layer_pairs(missing, gate, frozen, friction_cone_check=False)['passed']
    assert not retained_layer_pairs(replacement, gate, frozen, friction_cone_check=False)['passed']
    from tools.lib.so101_bimanual_native_contact import retained_material_patch
    redistributed = deepcopy(records)
    for i in (0, 2):
        # The original moving-contact node now also contacts the fixed jaw.
        redistributed[i]['particle'] = redistributed[i+1]['particle']
        redistributed[i]['particle_world_m'] = redistributed[i+1]['particle_world_m'][:]
    assert not retained_layer_pairs(redistributed, gate, frozen)['passed']
    assert retained_material_patch(redistributed, gate, frozen)['passed']
    assert not retained_material_patch(replacement, gate, frozen)['passed']
    assert not retained_material_patch(missing, gate, frozen)['passed']
    assert not retained_material_patch(oblique, gate, frozen)['passed']
    assert retained_material_patch(oblique, gate, frozen, friction_cone_check=False)['passed']
    assert retained_material_patch(redistributed, gate, frozen, friction_cone_check=False)['passed']
    assert not retained_material_patch(replacement, gate, frozen, friction_cone_check=False)['passed']
    assert not retained_material_patch(missing, gate, frozen, friction_cone_check=False)['passed']


def test_prelift_reserve_requires_both_jaws_on_both_qualified_layers():
    from copy import deepcopy
    from tools.lib.so101_bimanual_native_contact import pair_contact_reserve
    geometry = {'passed': True, 'layers': {str(h): {'passed': True,
        'actual_contacts': [{'penetration_m': .0004}, {'penetration_m': .0003}]}
        for h in (0, 1)}}
    assert pair_contact_reserve(geometry, .00025)['passed']
    shallow = deepcopy(geometry)
    shallow['layers']['0']['actual_contacts'][0]['penetration_m'] = .000024
    assert not pair_contact_reserve(shallow, .00025)['passed']
    missing = deepcopy(geometry); missing['layers']['1']['actual_contacts'].pop()
    assert not pair_contact_reserve(missing, .00025)['passed']
    geometry['passed'] = False
    assert not pair_contact_reserve(geometry, .00025)['passed']


def test_reserve_finds_deep_original_pair_despite_lower_id_shallow_pair():
    from copy import deepcopy
    from tools.lib.so101_mesh_pinch import ActualMeshPinchGate
    from tools.lib.so101_bimanual_native_contact import retained_material_patch, pair_contact_reserve
    contacts=[]
    for i in [0,1,3,4]:
        gap=.0029 if i in [0,3] else .002
        for fixed in [True,False]:
            contacts.append(dict(shape=('/Robot/left_gripper_link/TowelFixedJawCollider' if fixed else
                '/Robot/left_moving_jaw_link/moving_jaw_so101_v1'),particle=i,
                surface_world_m=[0 if fixed else 2*gap,0,0],surface_local_m=[-.0057,0,0],
                particle_world_m=[gap,0,0],normal_world=[1. if fixed else -1.,0,0],
                normal_local=[1. if fixed else -1.,0,0],penetration_m=.003-gap,
                signed_distance_m=gap,radius_m=.003,effective_friction_coefficient=.8))
    gate=ActualMeshPinchGate([[0,1,2],[3,4,5]])
    frozen={0:{'pair':[0,1],'triangle':0},1:{'pair':[3,4],'triangle':1}}
    assert not pair_contact_reserve(retained_material_patch(contacts,gate,frozen),.00075)['passed']
    result=retained_material_patch(contacts,gate,frozen,minimum_overlap_m=.00075)
    assert pair_contact_reserve(result,.00075)['passed']
    assert result['layers']['0']['opposing_contact_particle_pair']==[1,1]
    replaced=deepcopy(contacts)
    for c in replaced:
        if c['particle']==1:c['particle']=2
    assert not retained_material_patch(replaced,gate,frozen,minimum_overlap_m=.00075)['passed']
    oblique=deepcopy(contacts)
    for c in oblique:
        if c['particle']==1 and 'moving_jaw_link' in c['shape']:
            c['normal_world']=[-.1,.99498743710662,0];c['normal_local']=c['normal_world']
    assert not retained_material_patch(oblique,gate,frozen,minimum_overlap_m=.00075)['passed']
