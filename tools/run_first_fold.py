#!/usr/bin/env python3
"""Check the packaged evidence or run the preserved first-fold simulation."""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def read(path):
    return json.loads(Path(path).read_text())


def check_result(path):
    result = read(path)
    attachment = result['attachment']
    release = result['place_release']
    monitor = attachment['frictional_fold_monitor']
    vertices = result['final_cloth_shape_local_m_env_0']
    checks = {
        'finite_4096_node_cloth': len(vertices) == 4096 and all(
            len(point) == 3 and all(math.isfinite(x) for x in point) for point in vertices),
        'no_hardware_commands': result['motion_commands'] == 0,
        'no_scripted_cloth_attachment': all(attachment[key] is False for key in (
            'scripted_attachment_used', 'newton_state_constraint_used',
            'newton_nodal_kinematic_target_api_used', 'contact_gated_no_slip_retention_used')),
        'native_carry_complete': monitor['completed_carry_to_supported_fold'] is True
            and monitor['cloth_nodes_constrained'] is False,
        'fold_shape': release['shape_gate_passed'] is True,
        'material_alignment': release['strict_alignment_gate_passed'] is True
            and release['maximum_p95_paired_vertex_xy_error_m']
            <= release['maximum_p95_paired_vertex_xy_error_limit_m'],
        'half_fold': release['raw_underfold_gate_passed'] is True
            and bool(release['raw_underfold_observations']) and all(
                item['maximum_layer_fraction'] <= item['maximum_layer_fraction_limit']
                for item in release['raw_underfold_observations']),
        'released_from_jaws': release['minimum_release_patch_to_jaw_distance_m']
            >= release['minimum_release_patch_to_jaw_distance_limit_m'],
        'cloth_does_not_follow_open_jaws': release['maximum_release_patch_lift_m']
            <= release['maximum_release_patch_lift_limit_m'],
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise ValueError('Simulation result failed: ' + ', '.join(failed))
    return checks


def check_package():
    for name, expected in read(ROOT / 'config/package_sha256.json').items():
        path = ROOT / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Packaged input changed or missing: {name}')
    # The preserved runner resolves relative paths from the repository root.
    from tools.lib.towel_isaac_s0 import validate_s0_host_manifest
    from tools.lib.so101_gripper_geometry import load_gripper_geometry_candidate
    from tools.lib.so101_surface_matched_pad import load_surface_matched_pad
    validate_s0_host_manifest(read(ROOT / 'config/first_fold_scene.json'))
    load_gripper_geometry_candidate(ROOT / 'config/so101_gripper_geometry.candidate.json')
    load_surface_matched_pad(ROOT / 'artifacts/bimanual/planning/so101_surface_matched_pad_20260906/geometry.json', ROOT)
    for urdf in (ROOT / 'artifacts/bimanual/preview').glob('*.urdf'):
        for mesh in ET.parse(urdf).iter('mesh'):
            uri = mesh.attrib['filename']
            prefix = 'package://so101_description/'
            if not uri.startswith(prefix):
                raise ValueError(f'Unexpected mesh URI: {uri}')
            path = ROOT / 'ros2_ws/src/so101_description' / uri[len(prefix):]
            if not path.is_file():
                raise FileNotFoundError(path)
    verification = read(ROOT / 'results/first_fold/verification.json')
    result = ROOT / verification['result_path']
    if hashlib.sha256(result.read_bytes()).hexdigest() != verification['result_sha256']:
        raise ValueError('Stored result identity differs')
    return check_result(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true', help='Validate files and saved result; no Isaac Sim needed')
    mode.add_argument('--run', action='store_true', help='Compute the full simulation again')
    parser.add_argument('--gui', action='store_true', help='Display Isaac Sim during --run')
    parser.add_argument('--python', default=os.environ.get('ISAAC_PYTHON', sys.executable), help='Python in the Isaac Sim / Isaac Lab environment')
    parser.add_argument('--output', type=Path, help='New output directory (must not exist)')
    args = parser.parse_args()
    os.chdir(ROOT)
    checks = check_package()
    print(f'Package and stored simulation result verified ({len(checks)} result checks).', flush=True)
    if args.check:
        return 0
    output = args.output or ROOT / 'output' / datetime.now().strftime('first_fold_%Y%m%d_%H%M%S_%f')
    output = output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    recipe = read(ROOT / 'config/first_fold_recipe.json')
    command = [args.python, *recipe['arguments']]
    command[command.index('--output') + 1] = str(output / 'result.json')
    if args.gui:
        command.remove('--headless')
    environment = os.environ.copy()
    environment.update(recipe['environment'])
    (output / 'execution.json').write_text(json.dumps({'argv': command, 'environment': recipe['environment']}, indent=2) + '\n')
    print(f'Simulation output: {output}', flush=True)
    with (output / 'isaac.log').open('w') as log:
        completed = subprocess.run(command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT)
    if completed.returncode:
        raise RuntimeError(f'Isaac Sim exited with {completed.returncode}; see {output / "isaac.log"}')
    checks = check_result(output / 'result.json')
    (output / 'validation.json').write_text(json.dumps({'simulation_only': True, 'checks': checks}, indent=2) + '\n')
    print(f'First-fold simulation passed. Result: {output / "result.json"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
