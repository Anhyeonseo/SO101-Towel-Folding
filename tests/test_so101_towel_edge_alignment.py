import numpy as np
import pytest
from tools.lib.so101_towel_edge_alignment import mirrored_material_indices,aligned_pair_target


def test_mirrors_fold_material_coordinate_without_swapping_rows():
    assert mirrored_material_indices([3907,3971,259,323],64)==[3964,4028,316,380]
    ids=[0,63,64,4095]
    assert mirrored_material_indices(mirrored_material_indices(ids,64),64)==ids
    with pytest.raises(ValueError):mirrored_material_indices([4096],64)


def test_target_follows_observed_opposite_edge_preserving_rigid_pair_length():
    cloth=np.array([[x*.01,y*.01,0] for y in range(4) for x in range(4)])
    pair=np.array([[0,0,0],[0,.004,0]])
    target,indices=aligned_pair_target(cloth,[0,4],pair)
    assert indices==[3,7]
    np.testing.assert_allclose(target.mean(0),[.03,.005,.006])
    assert np.linalg.norm(target[1]-target[0])==pytest.approx(.004)
    cloth[indices,0]+=.02
    changed,_=aligned_pair_target(cloth,[0,4],pair)
    np.testing.assert_allclose(changed,target+[.02,0,0])


def test_anchor_plan_rejects_airborne_free_half_without_mutating_path():
    from tools.lib.so101_towel_edge_alignment import plan_observed_anchor_shift
    import copy
    class TranslationPlanner:
        def points(self,q):
            return np.array([q[:3],np.asarray(q[:3])+[0,.004,0]])
        def plan(self,start,target):
            q=np.r_[target[0],0.,0.]
            return q,{'passed':True}
    phases=[{'name':'first_form_l_05','joint_positions_rad':[0.]*12,
             'arm_joint_positions_rad':{'left':[0.]*5,'right':[0.]*5}}]
    original=copy.deepcopy(phases)
    cloth=np.array([[x*.01,y*.004,0] for y in range(4) for x in range(4)])
    cloth[[3,7],2]=.02
    result,report=plan_observed_anchor_shift(phases,{'left':TranslationPlanner(),'right':TranslationPlanner()},
        {'left':np.zeros(5),'right':np.zeros(5)},cloth,{'left':[0,4],'right':[8,12]})
    assert result is None and not report['passed']
    assert phases==original


def test_anchor_plan_moves_whole_remaining_arc_and_preserves_input():
    from tools.lib.so101_towel_edge_alignment import plan_observed_anchor_shift
    import copy
    class TranslationPlanner:
        def points(self,q):
            return np.array([q[:3],np.asarray(q[:3])+[0,.004,0]])
        def plan(self,start,target):
            return np.r_[target[0],0.,0.],{'passed':True}
    phases=[{'name':name,'joint_positions_rad':[0.]*12,
             'arm_joint_positions_rad':{'left':[0.]*5,'right':[0.]*5}}
            for name in ['first_form_l_05','first_form_l_06','first_gravity_laydown_01']]
    original=copy.deepcopy(phases)
    cloth=np.array([[x*.01,y*.004,0] for y in range(4) for x in range(4)])
    result,report=plan_observed_anchor_shift(phases,{'left':TranslationPlanner(),'right':TranslationPlanner()},
        {'left':np.zeros(5),'right':np.zeros(5)},cloth,{'left':[0,4],'right':[8,12]})
    assert report['passed'] and phases==original
    assert [p['joint_positions_rad'][0] for p in result]==pytest.approx([.015,.03,.03])
    assert [p['joint_positions_rad'][6] for p in result]==pytest.approx([.015,.03,.03])


def test_material_balance_measures_excess_length_and_ignores_corner_outliers():
    from tools.lib.so101_towel_edge_alignment import observed_fold_balance
    n=65;step=.3/(n-1);crease=34
    cloth=np.array([[.3-abs(c-crease)*step,r*step,0.] for r in range(n) for c in range(n)])
    report=observed_fold_balance(cloth)
    assert report['upper_minus_lower_length_m']==pytest.approx(4*step)
    assert report['advance_direction_xy']==pytest.approx([1.,0.])
    cloth[:n,0]+=.2
    assert observed_fold_balance(cloth)['upper_minus_lower_length_m']==pytest.approx(4*step)
    symmetric=np.array([[.3-abs(c-32)*step,r*step,0.] for r in range(n) for c in range(n)])
    assert observed_fold_balance(symmetric)['upper_minus_lower_length_m']==pytest.approx(0.,abs=1e-12)


def test_material_balance_rejects_an_unfolded_sheet():
    from tools.lib.so101_towel_edge_alignment import observed_fold_balance
    cloth=np.array([[c*.005,r*.005,0.] for r in range(16) for c in range(16)])
    with pytest.raises(ValueError,match='central fold'):
        observed_fold_balance(cloth)


def test_surface_drag_compensation_preserves_successful_half_slip_geometry():
    from tools.lib.so101_towel_edge_alignment import surface_drag_phase_offset
    delta = [.030, 0., 0.]
    # Accepted 2026-09-03 trajectory: +15 mm X / -15 mm Z at turnaround,
    # +30 mm X at release. These are geometry observations, not fit tolerances.
    np.testing.assert_allclose(surface_drag_phase_offset('first_form_l_09',delta),
                               [.015,0.,-.015])
    np.testing.assert_allclose(surface_drag_phase_offset('first_gravity_laydown_03',delta),
                               [.0225,0.,-.015*np.cos(np.pi/6)])
    np.testing.assert_allclose(surface_drag_phase_offset('first_gravity_overcenter_03',delta),
                               [.030,0.,0.])
    # Additional correction enters gradually after the observed L-04 state.
    np.testing.assert_allclose(surface_drag_phase_offset('first_form_l_05',delta),
                               [.005,0.,-.005])
    negative = surface_drag_phase_offset('first_form_l_09',[-.030,.010,0.])
    np.testing.assert_allclose(negative,[-.015,.010,.015])
    for index in range(7,10):
        offset = surface_drag_phase_offset(f'first_form_l_{index:02d}',delta,
                                           start_after_l_phase=6)
        np.testing.assert_allclose(offset,[.005*(index-6),0.,-.005*(index-6)])


def test_surface_drag_compensation_rejects_unknown_phase():
    from tools.lib.so101_towel_edge_alignment import surface_drag_phase_offset
    with pytest.raises(ValueError):
        surface_drag_phase_offset('first_gravity_retreat',[.030,0.,0.])
