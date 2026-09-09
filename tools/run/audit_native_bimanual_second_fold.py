#!/usr/bin/env python3
"""Sample a geometry-only bimanual candidate; never authorizes execution.

This screening checks open/closed/asymmetric jaw endpoints at contact and
transport poses. A passing screen still needs continuous path and jaw-envelope
verification, followed by native cloth contact validation.
"""
import argparse
import hashlib
import itertools
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.extend(['/opt/ros/jazzy/lib/python3.12/site-packages', '/usr/lib/python3/dist-packages'])
from tools.lib.so101_bimanual_mesh_audit import BimanualMeshAudit


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--table-manifest', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        ap.error('refusing overwrite')
    plan = json.loads(args.plan.read_text())
    if not plan['phases']:
        ap.error('plan has no solved phases')
    pad = json.loads(Path(plan['pad_path']).read_text())
    assert hashlib.sha256(Path(plan['pad_path']).read_bytes()).hexdigest() == plan['pad_sha256']
    table = json.loads(args.table_manifest.read_text())['worktable_geometry']
    audit = BimanualMeshAudit(ROOT, Path(plan['urdf_path']), pad, table)
    rows = []
    phases = [{'name': 'initial_clear', 'joint_positions_rad': plan['clear_model_rad']}] + plan['phases']
    for phase in phases:
        name = phase['name']
        envelope = name == 'second_bimanual_contact' or '_fold_' in name or name == 'second_bimanual_retreat'
        openings = list(itertools.product(('open_model_angle_rad', 'minimum_model_angle_rad'), repeat=2)) if envelope else [('open_model_angle_rad',) * 2]
        for left, right in openings:
            q = list(phase['joint_positions_rad'])
            q[5] = plan['closure']['left'][left]
            q[11] = plan['closure']['right'][right]
            rows.append({'phase': name, 'left_jaw': left, 'right_jaw': right, 'q_rad': q, **audit.state(q)})
    failures = [row for row in rows if not row['passed']]
    result = {'status': 'BIMANUAL_MESH_SCREEN_BRANCH' if failures else 'BIMANUAL_MESH_SAMPLES_PASS',
              'plan': str(args.plan.resolve()), 'plan_sha256': hashlib.sha256(args.plan.read_bytes()).hexdigest(),
              'geometry_hashes': audit.hashes, 'table': table,
              'table_manifest_sha256': hashlib.sha256(args.table_manifest.read_bytes()).hexdigest(),
              'mount_meshes': audit.mounts, 'object_count': len(audit.entries), 'pair_count': len(audit.pairs),
              'sample_count': len(rows), 'failed_sample_count': len(failures),
              'continuous_path_checked': False, 'continuous_jaw_envelope_checked': False,
              'cloth_contact_validated': False, 'one_flip_completed': False, 'samples': rows}
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('samples', 'geometry_hashes')}, indent=2))
    for row in failures:
        if row['left_jaw'] == row['right_jaw'] == 'open_model_angle_rad':
            print(row['phase'], [(x['kind'], x['pair'], round(x['distance_m']*1000, 3)) for x in row['failures']])


if __name__ == '__main__':
    main()
