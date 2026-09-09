#!/usr/bin/env python3
"""Read-only sampled cloth-surface intersection audit on the actual closed pad."""
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


def volume_intersections(mesh, local, triangles):
    """Sample vertex and triangle-interior penetration into a watertight pad."""
    weights = np.array([[i/10, j/10, (10-i-j)/10] for i in range(1, 9) for j in range(1, 10-i)])
    inside = mesh.contains(local)
    lo, hi = mesh.bounds
    verts = local[triangles]
    nearby = np.where(np.all(verts.max(axis=1) >= lo, axis=1) & np.all(verts.min(axis=1) <= hi, axis=1))[0]
    candidates = nearby[~inside[triangles[nearby]].any(axis=1)]
    found = []; examples = []; maximum_interior_depth = 0.
    if len(candidates):
        points = np.einsum('wv,tvc->twc', weights, verts[candidates])
        interior = mesh.contains(points.reshape(-1, 3)).reshape(len(candidates), -1)
        if interior.any():
            maximum_interior_depth = float(trimesh.proximity.closest_point(mesh, points[interior])[1].max())
        found = candidates[interior.any(axis=1)].tolist()
        for i in np.where(interior.any(axis=1))[0][:4]:
            j = np.where(interior[i])[0][0]
            examples.append({'triangle': int(candidates[i]), 'barycentric': weights[j].tolist(),
                             'inside_local_m': points[i, j].tolist()})
    ids = np.where(inside)[0]
    depth = trimesh.proximity.closest_point(mesh, local[ids])[1] if len(ids) else np.array([])
    return {'cloth_vertices_inside_pad': ids.tolist(),
        'vertex_depth_m': depth.tolist(), 'triangles_with_interior_inside_and_vertices_outside': found,
        'maximum_sampled_interior_depth_m': maximum_interior_depth,
        'examples': examples}


def audit(trial):
    report = json.loads((trial/'result.json').read_text())
    plan = json.loads((trial/'input.json').read_text())['plan']
    pad = json.loads(Path(plan['pad_path']).read_text())
    mesh = trimesh.Trimesh(vertices=pad['vertices_m'], faces=pad['faces'], process=True)
    if not mesh.is_watertight:
        raise ValueError('pad is not a closed volume')
    body = [i for i, label in enumerate(report['recording_body_labels'])
            if label.endswith('/left_gripper_link')]
    if len(body) != 1:
        raise ValueError(f'expected one left gripper body, got {body}')
    triangles = np.load(trial/'cloth_triangles.npy')
    result = {'scope': 'Actual pad volume; interior barycentric samples at 0.1 spacing. Positive findings prove surface intersection; absence is not continuous collision clearance. Not rubber compression or force measurement.',
              'stages': {}}
    stages = list(report['stages'])
    recording = None
    if (trial/'native_recording.npz').exists():
        recording = np.load(trial/'native_recording.npz')
        stages.append({'name': 'last_recorded_frame', 'time_s': float(recording['time_s'][-1])})
    for stage in stages:
        name = stage['name']
        state = ({'body_q': recording['body_q'][-1], 'particle_q': recording['cloth'][-1]}
                 if name == 'last_recorded_frame' else np.load(trial/(name+'_solver_contacts.npz')))
        transform = state['body_q'].reshape(-1, 7)[body[0]]
        local = (state['particle_q']-transform[:3])@Rotation.from_quat(transform[3:]).as_matrix()
        result['stages'][name] = {'time_s': stage['time_s'], **volume_intersections(mesh, local, triangles)}
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    args = parser.parse_args()
    result = audit(args.trial)
    (args.trial/'pad_intersection_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: {'inside_vertices': len(v['cloth_vertices_inside_pad']),
        'crossing_triangles': len(v['triangles_with_interior_inside_and_vertices_outside'])}
        for k, v in result['stages'].items()}))
