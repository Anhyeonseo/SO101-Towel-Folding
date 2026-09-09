#!/usr/bin/env python3
"""One bounded refinement of approach clearance; preserves S1 cloth and pad.

The alternate park is a simulation initialization candidate. The physical
transition from the original S1 arm state is deliberately not certified.
"""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages', '/usr/lib/python3/dist-packages'])
from tools.lib import towel_task_pose_planning as task
from tools.lib.so101_towel_edge_alignment import ContactPairPlanner


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        ap.error('refusing overwrite')
    p = json.loads(args.plan.read_text())
    rows = {r['name']: r for r in p['phases']}
    pad = json.loads(Path(p['pad_path']).read_text())
    closure = p['closure']['right']
    planner = ContactPairPlanner(ROOT, Path(p['urdf_path']), 'right',
        [p['planning_reference_in_gripper_m']], pad,
        closure['minimum_model_angle_rad'], closure['open_model_angle_rad'])
    planner.k._tcp_chain[-1].origin.xyz = p['planning_reference_in_gripper_m']
    row = rows['second_bimanual_contact']
    pose = task.TaskPose(**row['targets'][1])
    seed = np.array(row['joint_positions_rad'][6:11])
    angles = np.linspace(closure['minimum_model_angle_rad'], closure['open_model_angle_rad'], 9)

    def evaluate(q):
        return task.evaluate_task_pose(planner.k, pose, q, planner.lo, planner.hi)

    def constraints(q):
        e = evaluate(q)
        return np.array([
            (task.MAXIMUM_TCP_POSITION_ERROR_M-e['tcp_position_error_m'])*1000,
            task.MAXIMUM_JAW_YAW_ERROR_RAD-e['jaw_yaw_error_rad'],
            pose.maximum_approach_tilt_rad-e['approach_tilt_from_down_rad'],
            pose.maximum_finger_tilt_rad-e['finger_tilt_from_table_rad'],
            *[(planner.clearance(q, a)-.00075)*1000 for a in angles]])

    def loss(q):
        e = evaluate(q)
        return (e['tcp_position_error_m']/.0005)**2 + (e['jaw_yaw_error_rad']/.035)**2 + (e['approach_tilt_from_down_rad']/.35)**2 + .01*np.sum((q-seed)**2)

    fit = minimize(loss, seed, method='SLSQP',
        bounds=list(zip(planner.lo+.025, planner.hi-.025)),
        constraints=[{'type': 'ineq', 'fun': constraints}],
        options={'maxiter': 120, 'ftol': 1e-9})
    evaluation = evaluate(fit.x)
    minimum = min(planner.clearance(fit.x, a) for a in np.linspace(angles[0], angles[-1], 41))
    passed = evaluation['task_pose_pass'] and minimum >= .00075-1e-8
    result = copy.deepcopy(p)
    result.update(parent_plan=str(args.plan.resolve()),
        parent_plan_sha256=hashlib.sha256(args.plan.read_bytes()).hexdigest(),
        status='BIMANUAL_APPROACH_REFINEMENT_COLLISION_PENDING' if passed else 'BIMANUAL_APPROACH_REFINEMENT_BRANCH',
        refinement={'optimizer_message': str(fit.message), 'iterations': fit.nit,
                    'contact_task_pose': evaluation, 'minimum_table_clearance_m': minimum,
                    'passed': passed},
        s1_to_s2_arm_transition_validated=False, continuous_path_checked=False)
    if passed:
        clear = (rows['second_bimanual_departure_10_left']['joint_positions_rad'][:6]
                 + rows['second_bimanual_departure_15_right']['joint_positions_rad'][6:])
        result['original_clear_model_rad'] = result['clear_model_rad']
        result['clear_model_rad'] = clear
        result['initialization_scope'] = 'Alternate collision-screened simulation park; reaching it from original S1 joints remains unvalidated.'
        phases = []
        current = np.array(clear)
        # Joint-space departure avoids the unnecessary TCP-line elbow branch.
        for side, sl in [('right', slice(6, 11)), ('left', slice(0, 5))]:
            target = current.copy()
            target[sl] = rows[f'second_bimanual_departure_40_{side}']['joint_positions_rad'][sl]
            for i in range(1, 21):
                q = current+(target-current)*i/20
                phases.append({'name': f'second_bimanual_departure_{i:02d}_{side}',
                    'joint_positions_rad': q.tolist(), 'targets': [], 'attachment_event': None,
                    'task_pose_evaluations': [], 'transition_collision_checked': False})
            current = target
        contact = np.array(row['joint_positions_rad']); contact[6:11] = fit.x
        for i in range(1, 11):
            q = current + (contact-current)*i/10
            phases.append({'name': 'second_bimanual_contact' if i == 10 else f'second_bimanual_precontact_{i:02d}',
                'joint_positions_rad': q.tolist(), 'targets': row['targets'] if i == 10 else [],
                'attachment_event': None, 'transition_collision_checked': False,
                'task_pose_evaluations': [row['task_pose_evaluations'][0], evaluation] if i == 10 else []})
        phases += copy.deepcopy(p['phases'][90:101])
        phases.append({'name': 'second_bimanual_reobserve_clear', 'joint_positions_rad': clear,
            'targets': [], 'attachment_event': None, 'transition_collision_checked': False})
        result['phases'] = phases
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'status': result['status'], **result['refinement']}, indent=2))


if __name__ == '__main__':
    main()
