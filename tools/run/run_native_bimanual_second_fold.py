#!/usr/bin/env python3
"""One bounded, simulation-only bimanual trial from the saved native S1 shape.

Uses the unchanged S1 initializer and established material-retention gates.
Never attaches cloth nodes. Each hand freezes its own capture independently.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages', '/usr/lib/python3/dist-packages'])
from tools.lib.so101_bimanual_native_contact import (arm_contacts, both_layers_lifted,
    freeze_layer_pairs, retained_layer_pairs, frozen_pair_indices, pair_contact_reserve,
    retained_material_patch, observed_material_bilateral_contacts)
from tools.lib.so101_bimanual_mesh_audit import BimanualMeshAudit
from tools.lib.so101_mesh_pinch import ActualMeshPinchGate
from tools.lib.so101_contact_direction_feedback import contact_direction_score, propose_contact_translation
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics
from tools.lib.so101_s2_drive_policy import PositionDrivePolicy
from tools.lib.so101_bounded_press import JawDeflectionStop
from tools.lib.so101_local_primitive_observer import capture_witness, witness_motion
from tools.run.audit_r2_pad_intersection import volume_intersections
from tools.run.run_native_second_fold import (layered_gate, initial_core_if_ready,
    initial_surface_patch, retained_surface_contacts, core_region_retention,
    retention_closure_guard)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--output-dir', type=Path, required=True)
    ap.add_argument('--drive-mode', choices=('legacy_state_override', 'position'), default='position')
    ap.add_argument('--coupling-mode', choices=('one_way', 'two_way'), default='one_way')
    args = ap.parse_args()
    plan = json.loads(args.plan.read_text())
    if plan.get('bounded_press'):
        assert args.drive_mode == 'position' and args.coupling_mode == 'two_way'
    assert plan['status'] == 'BIMANUAL_NATIVE_PREFLIGHT_PASS' and not plan['sample_failures']
    base = ROOT / 'artifacts/bimanual/planning/so101_surface_matched_pad_20260906'
    recipe = json.loads((base / 'validated_native_recipe.json').read_text())
    for path, digest in {**recipe['input_sha256'], **plan['geometry_hashes'],
                         plan['source_result']: plan['source_result_sha256']}.items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest, path
    pad = json.loads(Path(plan['pad_path']).read_text())
    initial = np.array(json.loads(Path(plan['source_result']).read_text())['final_cloth_shape_local_m_env_0'])
    audit = BimanualMeshAudit(ROOT, Path(plan['urdf_path']), pad, plan['table'])
    assert audit.state(plan['initial_model_rad'])['passed']
    out = args.output_dir.resolve(); out.mkdir(parents=True, exist_ok=False)
    argv = list(recipe['argv'][1:])
    argv[argv.index('--output')+1] = str(out/'unused_s1_result.json')
    argv = [a for a in argv if a != '--compact-soft-contact-buffers']
    if '--newton-coupling-mode' in argv:
        argv[argv.index('--newton-coupling-mode')+1] = args.coupling_mode
    else:
        argv.extend(['--newton-coupling-mode', args.coupling_mode])
    (out/'executed_runner.py').write_bytes(Path(__file__).read_bytes())
    (out/'input.json').write_text(json.dumps({'plan': plan, 'scene_argv': argv,
        'drive_mode': args.drive_mode, 'coupling_mode': args.coupling_mode,
        'initializer_adapter_debt': 'second_fold_contact_only dispatch is selected after locked S1 import; not G0 compliant yet',
        'plan_sha256': hashlib.sha256(args.plan.read_bytes()).hexdigest(),
        'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'helper_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [ROOT/'tools/run/run_native_second_fold.py', ROOT/'tools/lib/so101_bimanual_native_contact.py', ROOT/'tools/lib/so101_bimanual_mesh_audit.py', ROOT/'tools/lib/so101_contact_direction_feedback.py', ROOT/'tools/lib/so101_s2_drive_policy.py', ROOT/'tools/lib/so101_bounded_press.py', ROOT/'tools/lib/so101_local_primitive_observer.py', ROOT/'tools/run/audit_r2_pad_intersection.py']}}, indent=2)+'\n')
    sys.argv = argv
    spec = importlib.util.spec_from_file_location('native_bimanual_s1_scene', ROOT/argv[0])
    module = importlib.util.module_from_spec(spec); sys.modules[spec.name] = module
    spec.loader.exec_module(module); env = vars(module)
    env['args'].second_fold_contact_only = True; env['args'].keep_open = False
    report = {'status': 'RUNNING', 'motion_commands': 0, 'motion_authorized': False,
        'drive_mode': args.drive_mode, 'coupling_mode': args.coupling_mode,
        'g1_passed': False, 'drive_trace': [],
        'cloth_nodes_constrained': False, 'one_flip_completed': False,
        'initialization': 'saved S1 nodal shape and S1 retreat target joints, zero velocities; not a full solver checkpoint',
        's1_to_s2_full_solver_checkpoint': False, 'stages': [], 'retention_trace': [],
        'source_result_sha256': plan['source_result_sha256'], 'pad_sha256': plan['pad_sha256']}
    recording = {k: [] for k in ['cloth', 'body_q', 'joints', 'time_s', 'phase']}
    started = time.monotonic()
    terminal_capture = None

    def save():
        report['elapsed_s'] = time.monotonic()-started
        (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')

    def experiment(*, scene, sim, robot, cloth, joint_ids, physics_dt_s, **unused):
        nonlocal terminal_capture
        import torch
        from scipy.spatial.transform import Rotation
        manager = env['NewtonManager']; model = manager.get_model()
        from isaaclab_contrib.deformable.coupled_mjwarp_vbd_manager import NewtonCoupledMJWarpVBDManager
        report['actual_coupling_mode'] = NewtonCoupledMJWarpVBDManager._coupling_mode
        assert report['actual_coupling_mode'] == args.coupling_mode
        report['active_model'] = {name: getattr(model, name).numpy().tolist() for name in
            ('joint_limit_lower', 'joint_limit_upper', 'joint_effort_limit', 'joint_velocity_limit')
            if hasattr(model, name) and getattr(model, name) is not None}
        report['contact_model_parameters'] = {name: float(getattr(model, name)) for name in
            ('soft_contact_ke', 'soft_contact_kd', 'soft_contact_mu')}
        gate = ActualMeshPinchGate(model.tri_indices.numpy())
        tensor = torch.tensor(initial, dtype=cloth.data.nodal_pos_w.torch.dtype, device=sim.device)
        state = torch.cat((tensor.unsqueeze(0)+scene.env_origins[:, None, :], torch.zeros_like(tensor).unsqueeze(0)), dim=-1)
        cloth.write_nodal_state_to_sim_index(state)
        row = torch.tensor([plan['initial_model_rad']], dtype=torch.float32, device=sim.device)
        zero = torch.zeros_like(row)
        drive = None
        if args.drive_mode == 'position':
            drive = PositionDrivePolicy(joint_ids, plan['initial_model_rad'],
                [env['gripper_candidate'].model_limits_rad(s) for s in ('left', 'right')], physics_dt_s)
            drive.initialize(robot, row, zero)
        else:
            env['write_scripted_arm_state_and_drive_targets'](robot, row, zero, joint_ids, initialize_arm_state=True)
        bodies = {}
        for side in ('left', 'right'):
            shapes = [i for i, label in enumerate(model.shape_label) if f'/{side}_' in label and 'TowelFixedJawCollider' in label]
            assert len(shapes) == 1
            bodies[side] = int(model.shape_body.numpy()[shapes[0]])
        report['recording_body_labels'] = list(model.body_label)
        report['contact_regions'] = plan['contact_regions']
        tick = 0; held = {}; pending = {}; stable = {'left': 0, 'right': 0}; last_contacts = []
        contacts_tick = -1; contacts_cache = []
        diagnostic_start_tick = None
        primitive_witness = None
        feedback = plan.get('contact_direction_feedback')
        feedback_offset = np.zeros(12)
        feedback_recovering = False
        feedback_pause_start = None
        feedback_kin = {}
        if feedback:
            limits = json.loads((ROOT/'config/bimanual_operational_limits.json').read_text())
            for side in ('left', 'right'):
                kin = GraspYawKinematics(Path(plan['urdf_path']), side+'_')
                feedback_kin[side] = (kin, kin._build_chain('workcell_base_link', side+'_gripper_link'))
        pair_mode = plan.get('capture_mode') == 'immutable_triangle_pair'
        retention_mode = plan.get('retention_mode', 'immutable_pair_roles')
        assert retention_mode in ('immutable_pair_roles', 'immutable_material_patch')
        observe_pairs = retained_material_patch if retention_mode == 'immutable_material_patch' else retained_layer_pairs
        report['retention_mode'] = retention_mode
        report['capture_mode'] = plan.get('capture_mode', 'legacy_individual_bilateral_core')
        np.save(out/'cloth_triangles.npy', model.tri_indices.numpy())

        def nodes(): return manager.get_state().particle_q.numpy()

        def local_nodes(side):
            transform = manager.get_state().body_q.numpy().reshape(-1, 7)[bodies[side]]
            return (nodes()-transform[:3]) @ Rotation.from_quat(transform[3:]).as_matrix()

        def snapshot():
            nonlocal contacts_tick, contacts_cache
            # Retention and capture inspect the same post-step geometry.
            if contacts_tick != tick:
                contacts_cache = env['newton_soft_contact_snapshot'](fresh_geometry=True)['jaw_contact_records']
                contacts_tick = tick
            return contacts_cache

        def event(name, **fields):
            # Original solver candidates are kept apart from fresh post-step geometry.
            c = manager.get_contacts()
            count = int(c.soft_contact_count.numpy()[0])
            np.savez_compressed(out/(name+'_solver_contacts.npz'),
                particle=c.soft_contact_particle.numpy()[:count], shape=c.soft_contact_shape.numpy()[:count],
                body_position=c.soft_contact_body_pos.numpy()[:count], normal=c.soft_contact_normal.numpy()[:count],
                particle_q=nodes(), body_q=manager.get_state().body_q.numpy())
            report['stages'].append({'name': name, 'time_s': tick*physics_dt_s,
                'commanded_joint_positions_rad': row[0].cpu().tolist(),
                'actual_joint_positions_rad': robot.data.joint_pos.torch[0, joint_ids].cpu().tolist(), **fields})
            np.save(out/(name+'.npy'), nodes())
            print('BIMANUAL_NATIVE '+name+' '+json.dumps(fields), flush=True); save()

        terminal_capture = lambda: event('terminal_stop', jaw_contacts=snapshot())

        def update_direction_feedback():
            nonlocal feedback_recovering, feedback_pause_start
            if not feedback or report.get('active_phase') not in ('lift_probe', 'lifted_hold', 'load_probe', 'load_probe_hold'):
                return
            if tick % 4:
                return
            scores = {s: contact_direction_score(arm_contacts(snapshot(), s), h['pairs']) for s, h in held.items()}
            if not feedback_recovering and max(scores.values()) >= feedback['trigger_score']:
                feedback_recovering = True; feedback_pause_start = tick
            if feedback_recovering and max(scores.values()) <= feedback['resume_score']:
                feedback_recovering = False; feedback_pause_start = None
            if not feedback_recovering:
                return
            if (tick-feedback_pause_start)*physics_dt_s > feedback['maximum_pause_s']:
                event('contact_direction_pause_timeout', scores=scores)
                raise RuntimeError('contact-direction feedback could not recover within bounded pause')
            for si, side in enumerate(('left', 'right')):
                if scores[side] <= feedback['resume_score']:
                    continue
                contacts = arm_contacts(snapshot(), side)
                proposal = propose_contact_translation(contacts, held[side]['pairs'], feedback['maximum_step_m'])
                record = {'time_s': tick*physics_dt_s, 'side': side, **proposal, 'applied': False}
                report.setdefault('direction_feedback_trace', []).append(record)
                if not proposal['accepted']:
                    continue
                kin, chain = feedback_kin[side]; sl = slice(si*6, si*6+5)
                # Predict the incremental correction only. Including servo
                # tracking residual would attribute nominal motion to feedback.
                q = row[0, sl].cpu().numpy().copy()
                def fk(qv): return kin._compose(chain, dict(zip(kin.arm_joints, qv)))
                R, t = fk(q); jac = np.empty((6, 5))
                for j in range(5):
                    qp = q.copy(); qp[j] += 1e-5; Rp, tp = fk(qp)
                    jac[:3, j] = (tp-t)/1e-5
                    jac[3:, j] = .1*Rotation.from_matrix(Rp@R.T).as_rotvec()/1e-5
                rhs = np.r_[proposal['translation_world_m'], np.zeros(3)]
                dq = np.linalg.lstsq(jac, rhs, rcond=1e-6)[0]
                cap = feedback['maximum_joint_speed_rad_s']*4*physics_dt_s
                dq *= min(1., cap/max(abs(dq).max(), 1e-12))
                candidate = row[0].cpu().numpy().copy(); candidate[sl] += dq
                Rc, tc = fk(candidate[sl]); predicted_shift = tc-t
                Rbase, tbase = fk(candidate[sl]-feedback_offset[sl]-dq)
                tracked_ids = sorted({i for ids in held[side]['core'].values() for i in ids})
                tracked = nodes()[tracked_ids]
                total_motion = np.linalg.norm((tracked-tc)@Rc-(tracked-tbase)@Rbase, axis=1).max()
                original_motion = np.linalg.norm((tracked-tc)@Rc-held[side]['local'][tracked_ids], axis=1).max()
                qlimits = limits['arms'][side]
                names = ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll')
                legal = all(qlimits[n]['minimum_urad']*1e-6 <= v <= qlimits[n]['maximum_urad']*1e-6 for n, v in zip(names, candidate[sl]))
                predicted = contact_direction_score(contacts, held[side]['pairs'], predicted_shift)
                record.update(joint_delta_rad=dq.tolist(), predicted_score_after_ik=predicted,
                              cumulative_point_correction_m=float(total_motion),
                              predicted_original_material_motion_m=float(original_motion))
                if not legal or original_motion > .003 or total_motion > feedback['maximum_total_correction_m'] or predicted >= scores[side]-.001:
                    record['rejection'] = 'joint limit, cumulative correction or no predicted improvement'; continue
                if not audit.state(candidate)['passed']:
                    record['rejection'] = 'mesh clearance'; continue
                row[0, sl] = torch.tensor(candidate[sl], dtype=row.dtype, device=row.device)
                feedback_offset[sl] += dq; record['applied'] = True

        def step():
            nonlocal tick, last_contacts, diagnostic_start_tick
            if drive:
                drive.command(robot, row, row[0].cpu().numpy())
                report['joint_state_initializations'] = drive.state_writes
                report['position_commands'] = drive.commands
            else:
                env['write_scripted_arm_state_and_drive_targets'](robot, row, zero, joint_ids)
            scene.write_data_to_sim(); sim.step(); scene.update(physics_dt_s); tick += 1
            if not torch.isfinite(cloth.data.nodal_pos_w.torch).all():
                raise RuntimeError('nonfinite cloth')
            if tick % 8 == 0:
                recording['cloth'].append(nodes().copy()); recording['body_q'].append(manager.get_state().body_q.numpy().copy())
                recording['joints'].append(robot.data.joint_pos.torch[0, joint_ids].cpu().numpy().copy())
                recording['time_s'].append(tick*physics_dt_s); recording['phase'].append(report.get('active_phase', 'initialization'))
            if tick % 4 == 0:
                actual = robot.data.joint_pos.torch[0, joint_ids].cpu().numpy()
                report['drive_trace'].append({'time_s': tick*physics_dt_s,
                    'phase': report.get('active_phase', 'initialization'),
                    'target_rad': row[0].cpu().tolist(), 'actual_rad': actual.tolist(),
                    # The current/output buffer has been cleared by coupling.
                    # Keep both buffers raw; neither is a calibrated grip force.
                    'current_state_body_wrench_buffer': manager.get_state().body_f.numpy().tolist(),
                    'alternate_state_body_wrench_buffer': manager._state_1.body_f.numpy().tolist(),
                    'velocity_rad_s': robot.data.joint_vel.torch[0, joint_ids].cpu().tolist()})
                if not np.isfinite(actual).all():
                    raise RuntimeError('nonfinite robot joints')
                checked = audit.state(actual)
                report['last_actual_mesh_check'] = checked
                if not checked['passed']:
                    event('actual_mesh_branch', failures=checked['failures'])
                    raise RuntimeError('actual arm configuration violated mesh clearance')
                if primitive_witness is not None:
                    measured = witness_motion(primitive_witness['local'], local_nodes('left'),
                        primitive_witness['world'], nodes(), primitive_witness['layers'])
                    report.setdefault('primitive_material_trace', []).append({'time_s': tick*physics_dt_s, **measured})
                    if not measured['within_motion_bound']:
                        raise RuntimeError('local lift diagnostic exceeded original-material 3 mm motion bound')
            if held:
                last_contacts = snapshot()
                for si, side in enumerate(('left', 'right')):
                    if side not in held: continue
                    h = held[side]; contacts = arm_contacts(last_contacts, side)
                    geometry = (observe_pairs(contacts, gate, h['pairs']) if pair_mode else
                        retained_surface_contacts(contacts, h['region'], allow_contact_redistribution=True))
                    measured = core_region_retention(local_nodes(side), h['local'], h['core'], h['region'], geometry)
                    diagnostic_retention = None
                    diagnostic_scope = (not plan.get('diagnostic_actual_contacts_only') or
                        report.get('active_phase') in ('lift_probe', 'lifted_hold'))
                    if pair_mode and plan.get('diagnostic_cone_only_continuation') and not geometry['passed'] and diagnostic_scope:
                        observed = (observed_material_bilateral_contacts(contacts, h['pairs'])
                            if plan.get('diagnostic_actual_contacts_only') else
                            observe_pairs(contacts, gate, h['pairs'], friction_cone_check=False))
                        diagnostic_retention = core_region_retention(local_nodes(side), h['local'],
                            h['core'], h['region'], observed)
                        if diagnostic_retention['passed'] and diagnostic_start_tick is None:
                            diagnostic_start_tick = tick
                            report['strict_contact_criterion_failed'] = True
                            event('diagnostic_cone_only_started', side=side, strict_geometry=geometry,
                                observed_contact_geometry=observed, jaw_contacts=last_contacts)
                    h['loss'] = 0 if measured['passed'] else h['loss']+1
                    guard = retention_closure_guard(contacts, h['region'])
                    report['retention_trace'].append({'time_s': tick*physics_dt_s, 'phase': report.get('active_phase'),
                        'side': side, **measured, 'closure_guard': guard,
                        'diagnostic_contact_retention': diagnostic_retention})
                    if (not measured['passed'] and guard['allow_additional_closure']
                            and not plan.get('diagnostic_cone_only_continuation')):
                        row[0, si*6+5] = max(h['minimum'], float(row[0, si*6+5])-.02*physics_dt_s)
                    if h['loss']*physics_dt_s > .02:
                        if diagnostic_retention and diagnostic_retention['passed']:
                            if (tick-diagnostic_start_tick)*physics_dt_s <= 3.:
                                continue
                            report['last_jaw_contacts'] = last_contacts
                            event('diagnostic_time_limit', side=side)
                            raise RuntimeError('cone-only diagnostic observation exceeded 3 seconds')
                        report['last_jaw_contacts'] = last_contacts
                        event('retention_branch_'+side, measured=measured)
                        raise RuntimeError(side+' lost immutable two-layer retention for over 20 ms')
                update_direction_feedback()

        def hold(name, seconds):
            report['active_phase'] = name
            for _ in range(round(seconds/physics_dt_s)): step()

        def move(phase):
            report['active_phase'] = phase['name']; target = np.array(phase['q_rad'])
            press = plan.get('bounded_press')
            press = press if press and press['phase'] == phase['name'] else None
            stop = JawDeflectionStop(**press['encoder_stop']) if press else None
            pressed = False
            begin = row.clone(); count = max(2, round(phase['seconds']/physics_dt_s))
            initial_offset = feedback_offset.copy(); i = 0; elapsed_ticks = 0
            def observe_press():
                observed = stop.observe(tick*physics_dt_s, float(row[0, 5]),
                    float(robot.data.joint_pos.torch[0, joint_ids[5]]))
                report.setdefault('press_trace', []).append({'time_s': tick*physics_dt_s, **observed})
                return observed
            while i < count or feedback_recovering:
                elapsed_ticks += 1
                if press and elapsed_ticks*physics_dt_s > press['maximum_seconds']:
                    raise RuntimeError('bounded descent exceeded time budget')
                can_advance = not press or env['maximum_arm_target_residual_rad'](robot, row, joint_ids) < press['maximum_command_lead_rad']
                if not feedback_recovering and can_advance:
                    i += 1
                    for sl in (slice(0, 5), slice(6, 11)):
                        nominal_begin = begin[0, sl]-torch.tensor(initial_offset[sl], dtype=row.dtype, device=row.device)
                        row[0, sl] = nominal_begin+(torch.tensor(target[sl], dtype=row.dtype, device=row.device)-nominal_begin)*i/count+torch.tensor(feedback_offset[sl], dtype=row.dtype, device=row.device)
                step()
                if stop:
                    observed = observe_press()
                    if observed['stop_descent']:
                        # Freeze the existing drive target, never the physical state.
                        pressed = True
                        event('descent_contact_response', **observed,
                              commanded_fraction=i/count, jaw_contacts=snapshot())
                        break
            for _ in range(round(.5/physics_dt_s)):
                if (not stop or pressed) and env['maximum_arm_target_residual_rad'](robot, row, joint_ids) < .001: break
                step()
                if stop and not pressed:
                    observed = observe_press()
                    if observed['stop_descent']:
                        pressed = True
                        event('descent_contact_response', **observed,
                              commanded_fraction=i/count, jaw_contacts=snapshot())
            if stop and not pressed:
                raise RuntimeError('bounded descent ended without sustained jaw encoder response')
            env['require_arm_target_reached'](robot, row, joint_ids, phase['name'])
            event(phase['name'])

        hold('initialization', .2)
        drift = float(np.linalg.norm(nodes()-initial, axis=1).max())
        event('initialized', maximum_node_drift_m=drift)
        if drift > .005: raise RuntimeError('saved S1 shape drifted by over 5 mm before approach')
        phases = {p['name']: p for p in plan['phases']}
        for phase in plan['phases']:
            if phase['name'] == 'lift_probe': break
            move(phase)
        hold('preclose_hold', .15)
        event('preclose', jaw_contacts=snapshot(), maximum_node_motion_from_s1_m=float(np.linalg.norm(nodes()-initial, axis=1).max()))
        formation = plan.get('u_pinch_diagnostic')
        if formation:
            # A separate formation observation, never a replacement grasp gate.
            # Record the entire gather/close motion before deciding whether a
            # four-sheet U actually exists. No points are attached or rebased.
            assert formation['side'] == 'left'
            assert not held and not pending
            start_angle = float(row[0, 5])
            end_angle = plan['closure']['left']['minimum_model_angle_rad']
            count = max(2, round(formation['closure_seconds']/physics_dt_s))
            report['active_phase'] = 'u_pinch_closure'
            for i in range(1, count+1):
                row[0, 5] = start_angle+(end_angle-start_angle)*i/count
                step()
                if i in {round(count*f) for f in (.25, .5, .75, 1.)}:
                    event('u_pinch_closure_'+str(i), jaw_contacts=snapshot(),
                          commanded_angle_rad=float(row[0, 5]))
            hold('u_pinch_closed_hold', formation['hold_seconds'])
            report['last_jaw_contacts'] = snapshot()
            event('u_pinch_formation_observed', jaw_contacts=snapshot())
            probe = plan.get('local_primitive_diagnostic')
            if probe:
                import trimesh
                from scipy.optimize import least_squares
                mesh = trimesh.Trimesh(vertices=pad['vertices_m'], faces=pad['faces'], process=True)
                crossing = volume_intersections(mesh, local_nodes('left'), model.tri_indices.numpy())
                witness = capture_witness(snapshot(), row=plan['contact_regions']['left']['row'],
                    columns=plan['contact_regions']['left']['columns'])
                event('local_primitive_capture_check', pad_intersection=crossing, witness=witness)
                if crossing['cloth_vertices_inside_pad'] or crossing['triangles_with_interior_inside_and_vertices_outside']:
                    raise RuntimeError('pad interpenetration blocks diagnostic lift')
                if not witness['candidate_present']:
                    raise RuntimeError('no two-half material witness on both jaws; diagnostic lift blocked')
                rest = row[0].cpu().numpy().copy()
                kin = GraspYawKinematics(Path(plan['urdf_path']), 'left_')
                chain = kin._build_chain('workcell_base_link', 'left_gripper_link')
                def fk(q): return kin._compose(chain, dict(zip(kin.arm_joints, q)))
                R0, t0 = fk(rest[:5]); destination = t0+np.array([0, 0, probe['lift_m']])
                limits = json.loads((ROOT/'config/bimanual_operational_limits.json').read_text())['arms']['left']
                names = ('base', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll')
                bounds = ([limits[n]['minimum_urad']*1e-6 for n in names],
                          [limits[n]['maximum_urad']*1e-6 for n in names])
                def residual(q):
                    R, t = fk(q)
                    return np.r_[t-destination, .1*Rotation.from_matrix(R@R0.T).as_rotvec()]
                solved = least_squares(residual, rest[:5], bounds=bounds, xtol=1e-12, ftol=1e-12, gtol=1e-12)
                if np.linalg.norm(residual(solved.x)) > 1e-5:
                    raise RuntimeError('vertical diagnostic lift IK residual too large')
                lifted = rest.copy(); lifted[:5] = solved.x
                for fraction in np.linspace(0, 1, 41):
                    if not audit.state(rest+(lifted-rest)*fraction)['passed']:
                        raise RuntimeError('diagnostic lift path mesh check failed')
                primitive_witness = {'local': local_nodes('left').copy(), 'world': nodes().copy(),
                                     'layers': witness['layers']}
                move({'name': 'local_lift', 'q_rad': lifted.tolist(), 'seconds': probe['move_seconds']})
                hold('local_lift_hold', probe['hold_seconds'])
                measured = witness_motion(primitive_witness['local'], local_nodes('left'),
                    primitive_witness['world'], nodes(), primitive_witness['layers'])
                event('local_lift_observed', material=measured)
                if not measured['minimum_lift_reached']:
                    raise RuntimeError('both original halves did not lift by 5 mm')
                move({'name': 'local_lower', 'q_rad': rest.tolist(), 'seconds': probe['move_seconds']})
                hold('local_supported_hold', .3)
                if float(np.mean(nodes()[:, 2] <= .006)) < .25:
                    raise RuntimeError('insufficient table support before diagnostic release')
                primitive_witness = None
                start = float(row[0, 5]); report['active_phase'] = 'local_open'
                count = round(1.5/physics_dt_s)
                for i in range(1, count+1):
                    row[0, 5] = start+(plan['closure']['left']['open_model_angle_rad']-start)*i/count
                    step()
                hold('local_open_hold', .3)
                released = nodes().copy()
                move({'name': 'local_retreat', 'q_rad': lifted.tolist(), 'seconds': probe['move_seconds']})
                hold('local_release_hold', probe['hold_seconds'])
                release_lifts = {h: float(np.median(nodes()[ids, 2]-released[ids, 2])) for h, ids in witness['layers'].items()}
                event('local_release_observed', material_rise_after_open_m=release_lifts,
                      pad_intersection=volume_intersections(mesh, local_nodes('left'), model.tri_indices.numpy()))
                if any(height > .003 for height in release_lifts.values()):
                    raise RuntimeError('original material still carried upward after opening')
                report['local_primitive_diagnostic_completed'] = True
                # This diagnostic cannot certify U layering, calibrated forces,
                # negative physical trials, or the complete R2 G1 contract.
            report['status'] = 'U_PINCH_FORMATION_DIAGNOSTIC_COMPLETED'
            report['diagnostic_only'] = True
            report['u_four_sheet_capture_validated'] = False
            report['bimanual_lift_validated'] = False
            return 0
        report['active_phase'] = 'closure'
        seating = plan.get('right_seating')
        seating_done = seating is None
        seating_begin = None; seating_ticks = 0
        defer_capture = bool(seating and plan.get('capture_after_seating'))
        count = round((1.25 + (seating['seconds']+.6 if seating else 0.) +
            (.8 if defer_capture else 0.) + (.25 if pair_mode else 0.))/physics_dt_s)
        for i in range(1, count+1):
            for si, side in enumerate(('left', 'right')):
                if side in held or side in pending: continue
                if side == 'left' and defer_capture and not seating_done: continue
                c = plan['closure'][side]
                speed = (c['open_model_angle_rad']-c['minimum_model_angle_rad'])/1.25
                next_angle = max(c['minimum_model_angle_rad'], float(row[0, si*6+5])-speed*physics_dt_s)
                if side == 'right' and not seating_done:
                    next_angle = max(next_angle, seating['maximum_opening_rad'])
                row[0, si*6+5] = next_angle
            if seating and not seating_done:
                if 'right' in held:
                    seating_done = True  # An established grasp is never repositioned to force the witness.
                elif 'right' not in pending and float(row[0, 11]) <= seating['maximum_opening_rad']+1e-7:
                    if seating_begin is None:
                        seating_begin = row[0, 6:11].clone()
                    seating_ticks += 1
                    fraction = min(1., seating_ticks*physics_dt_s/seating['seconds'])
                    target = torch.tensor(seating['q_rad'], dtype=row.dtype, device=sim.device)
                    row[0, 6:11] = seating_begin+(target-seating_begin)*fraction
                    if fraction == 1. and env['maximum_arm_target_residual_rad'](robot, row, joint_ids) < .002:
                        seating_done = True
            step(); last_contacts = snapshot(); evidence = {}
            for si, side in enumerate(('left', 'right')):
                if side in held: continue
                region = plan['contact_regions'][side]; contacts = arm_contacts(last_contacts, side)
                e = layered_gate(contacts, gate, row=region['row'], columns=region['columns'])
                if pair_mode:
                    if seating and not seating_done and (side == 'right' or defer_capture):
                        evidence[side] = e
                        continue
                    if side not in pending and e['passed']:
                        pairs = freeze_layer_pairs(e); core = frozen_pair_indices(pairs)
                        pending[side] = {'pairs': pairs, 'core': core, 'region': core,
                            'local': local_nodes(side).copy(), 'world': nodes().copy(),
                            'loss': 0, 'stable': 0, 'minimum': max(plan['closure'][side]['minimum_model_angle_rad'], float(row[0, si*6+5])-.04)}
                    if side in pending:
                        candidate = pending[side]
                        e = retained_layer_pairs(contacts, gate, candidate['pairs'])
                        measured = core_region_retention(local_nodes(side), candidate['local'],
                            candidate['core'], candidate['region'], e)
                        if not measured['passed']:
                            del pending[side]
                        else:
                            candidate['stable'] += 1
                            if candidate['stable'] >= 8:
                                held[side] = pending.pop(side)
                                event('captured_'+side, core=held[side]['core'], pairs=held[side]['pairs'],
                                    model_angle_rad=float(row[0, si*6+5]), capture_origin_reset_after_confirmation=False)
                    evidence[side] = e
                    continue
                evidence[side] = e; stable[side] = stable[side]+1 if e['passed'] else 0
                if stable[side] < 8: continue
                core = initial_core_if_ready(contacts, e, gate, row=region['row'])
                if core is None: continue
                columns = {h: int(e['layers'][str(h)]['selected_distinct_particles'][0]) % 64 for h in (0, 1)}
                frozen = initial_surface_patch(contacts, columns=columns, row=region['row'])
                if not all(set(core[h]).issubset(frozen[h]) for h in (0, 1)):
                    raise RuntimeError('core outside frozen contact patch')
                held[side] = {'core': core, 'region': frozen, 'local': local_nodes(side), 'world': nodes().copy(),
                    'loss': 0, 'minimum': max(plan['closure'][side]['minimum_model_angle_rad'], float(row[0, si*6+5])-.04)}
                event('captured_'+side, core=core, region=frozen, gate=e, model_angle_rad=float(row[0, si*6+5]))
            if i % 12 == 0: report.setdefault('closure_trace', []).append({'fraction': i/count, 'evidence': evidence})
            if len(held) == 2: break
        report['last_jaw_contacts'] = last_contacts
        if len(held) != 2:
            event('closure_branch', captured_arms=list(held), evidence=evidence)
            raise RuntimeError('safe closure ended before both arms captured both S1 layers')
        stabilization = plan.get('prelift_stabilization')
        if stabilization:
            sides = stabilization.get('sides', [stabilization.get('side', 'left')])
            assert pair_mode and sides and len(set(sides)) == len(sides)
            assert set(sides).issubset({'left', 'right'})
            report['active_phase'] = 'prelift_stabilization'
            joint = {'left': 5, 'right': 11}
            stable_ticks = {side: 0 for side in sides}
            start_angles = {side: float(row[0, joint[side]]) for side in sides}
            stop_angles = {side: max(held[side]['minimum'],
                start_angles[side]-stabilization['maximum_closure_rad']) for side in sides}
            for _ in range(round(stabilization['timeout_s']/physics_dt_s)):
                reserves = {}; geometries = {}; guards = {}
                for side in sides:
                    h = held[side]; contacts = arm_contacts(snapshot(), side)
                    geometry = observe_pairs(contacts, gate, h['pairs'])
                    reserve_geometry = geometry
                    if stabilization.get('all_original_material_pairs'):
                        assert retention_mode == 'immutable_material_patch'
                        reserve_geometry = retained_material_patch(contacts, gate, h['pairs'],
                            minimum_overlap_m=stabilization['minimum_overlap_m'])
                    if plan.get('diagnostic_cone_only_continuation') and not geometry['passed']:
                        reserve_geometry = retained_layer_pairs(contacts, gate, h['pairs'], friction_cone_check=False)
                    reserve = pair_contact_reserve(reserve_geometry, stabilization['minimum_overlap_m'])
                    guard = retention_closure_guard(contacts, h['region'])
                    reserves[side] = reserve; geometries[side] = reserve_geometry; guards[side] = guard
                    stable_ticks[side] = stable_ticks[side]+1 if reserve['passed'] else 0
                    report.setdefault('stabilization_trace', []).append({
                        'time_s': tick*physics_dt_s, 'side': side, **reserve, 'closure_guard': guard,
                        'strict_geometry_passed': geometry['passed'],
                        'model_angle_rad': float(row[0, joint[side]])})
                if all(v*physics_dt_s >= stabilization['stable_s'] for v in stable_ticks.values()):
                    event('prelift_stabilized', reserve_by_arm=reserves,
                        additional_closure_by_arm_rad={side: start_angles[side]-float(row[0, joint[side]]) for side in sides},
                        jaw_contacts=snapshot())
                    break
                for side in sides:
                    if not reserves[side]['passed'] and (geometries[side]['passed'] or
                            (stabilization.get('all_original_material_pairs') and
                             observe_pairs(arm_contacts(snapshot(), side), gate, held[side]['pairs'])['passed'])):
                        if not guards[side]['allow_additional_closure'] or float(row[0, joint[side]]) <= stop_angles[side]+1e-7:
                            report['last_jaw_contacts'] = snapshot()
                            event('prelift_stabilization_branch', side=side,
                                reserve_by_arm=reserves, closure_guard=guards[side])
                            raise RuntimeError(side+' contact reserve insufficient at guarded prelift closure limit')
                        row[0, joint[side]] = max(stop_angles[side],
                            float(row[0, joint[side]])-stabilization['speed_rad_s']*physics_dt_s)
                step()
            else:
                report['last_jaw_contacts'] = snapshot()
                event('prelift_stabilization_timeout', reserve_by_arm=reserves)
                raise RuntimeError('contact reserve did not stabilize within bounded prelift interval')
        if plan.get('prelift_alignment'):
            move(plan['prelift_alignment'])
            event('prelift_alignment_contacts', jaw_contacts=snapshot())
        hold('post_capture_hold', .15); move(phases['lift_probe'])
        lifted = both_layers_lifted({s: h['world'] for s, h in held.items()}, nodes(), {s: h['core'] for s, h in held.items()})
        event('lift_check', **lifted)
        if not lifted['passed']: raise RuntimeError('both layers at both hands must rise at least 5 mm')
        hold('lifted_hold', .4)
        if report.get('strict_contact_criterion_failed'):
            report['last_jaw_contacts'] = snapshot()
            event('diagnostic_lift_and_hold_completed', jaw_contacts=snapshot())
            report['status'] = 'BIMANUAL_DIAGNOSTIC_LIFT_HELD_STRICT_CRITERION_FAILED'
            return 0
        event('bimanual_grasp_probe_passed')
        report['bimanual_lift_validated'] = True
        if plan.get('stop_after_lift_hold'):
            report['last_jaw_contacts'] = snapshot()
            event('lift_hold_probe_completed', jaw_contacts=snapshot())
            report['status'] = 'BIMANUAL_NATIVE_LIFT_HOLD_PROBE_PASSED'
            return 0
        if 'load_probe' in phases:
            move(phases['load_probe']); hold('load_probe_hold', .15)
            report['last_jaw_contacts'] = snapshot()
            event('load_probe_completed', jaw_contacts=snapshot())
            if plan.get('stop_after_grasp_probe'):
                report['status'] = 'BIMANUAL_NATIVE_GRASP_HOLD_LOAD_PROBE_PASSED'
                return 0
        for phase in plan['phases']:
            if phase['name'].startswith('fold_'): move(phase)
        hold('laydown_hold', .3)
        support = float(np.mean(nodes()[:, 2] <= .006))
        event('laydown', supported_fraction=support)
        if support < .25: raise RuntimeError('less than 25 percent of cloth supported before release')
        held.clear(); begin = row.clone(); report['active_phase'] = 'release'
        for i in range(1, 61):
            for si, side in enumerate(('left', 'right')):
                row[0, si*6+5] = begin[0, si*6+5]+(plan['closure'][side]['open_model_angle_rad']-begin[0, si*6+5])*i/60
            step()
        hold('release_hold', .25); move(phases['retreat'])
        np.save(out/'final_cloth.npy', nodes())
        report['status'] = 'BIMANUAL_NATIVE_RELEASED_SHAPE_REVIEW_REQUIRED'
        event('released_shape', maximum_height_m=float(nodes()[:, 2].max()+.005))
        return 0

    env['run_second_fold_contact_only'] = experiment
    try: env['run']()
    except Exception as error:
        report['status'] = 'BIMANUAL_NATIVE_BRANCH_STOPPED'; report['reason'] = str(error)
        if terminal_capture is not None:
            try: terminal_capture()
            except Exception as capture_error: report['terminal_capture_error'] = str(capture_error)
        traceback.print_exc(); print('BIMANUAL_NATIVE_BRANCH '+str(error), flush=True)
    finally:
        if recording['time_s']:
            np.savez_compressed(out/'native_recording.npz', **{k: np.asarray(v) for k, v in recording.items()})
            report['recording'] = {'path': str(out/'native_recording.npz'), 'frames': len(recording['time_s']), 'physics_recomputation_required_for_playback': False}
        exit_code = 0 if report['status'] in ('BIMANUAL_NATIVE_RELEASED_SHAPE_REVIEW_REQUIRED',
            'BIMANUAL_NATIVE_GRASP_HOLD_LOAD_PROBE_PASSED', 'U_PINCH_FORMATION_DIAGNOSTIC_COMPLETED') else 2
        report['process_exit_code'] = exit_code
        save(); env['simulation_app'].close(exit_code=exit_code)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
