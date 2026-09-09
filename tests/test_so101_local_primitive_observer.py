import numpy as np
from tools.lib.so101_local_primitive_observer import capture_witness, witness_motion


def contact(particle, jaw, distance=.001):
    return {'particle': particle, 'shape': '/left_gripper_link/'+jaw,
            'signed_distance_m': distance, 'penetration_m': .003-distance,
            'surface_local_m': [-.0057, 0, 0]}


def test_empty_single_layer_and_embedded_centres_are_not_candidates():
    fixed = contact(62*64+3, 'TowelFixedJawCollider')
    moving = contact(62*64+60, 'left_moving_jaw_link/mesh')
    for contacts in ([], [fixed], [fixed, dict(moving, signed_distance_m=-.001)]):
        assert not capture_witness(contacts, row=62, columns=[3,60])['candidate_present']
    result = capture_witness([fixed,moving], row=62, columns=[3,60])
    assert result['candidate_present'] and not result['four_sheet_grasp_validated']


def test_world_lift_alone_cannot_hide_sliding_in_hand_frame():
    original = np.zeros((2,3)); current_world = original+[0,0,.008]
    moved_local = original.copy(); moved_local[1,0] = .004
    result = witness_motion(original,moved_local,original,current_world,{0:[0],1:[1]})
    assert result['minimum_lift_reached'] and not result['within_motion_bound']
