"""Oracle-only material witness for a bounded lift diagnostic, not U-grasp proof."""
import numpy as np


def capture_witness(contacts, *, row, columns):
    layers = {0: set(), 1: set()}
    jaws = set()
    for contact in contacts:
        if '/left_' not in contact['shape']:
            continue
        particle = int(contact['particle']); half = int(particle % 64 >= 32)
        if abs(particle//64-row) > 3 or abs(particle % 64-columns[half]) > 3:
            continue
        if not (contact['penetration_m'] > 0 and contact['signed_distance_m'] >= 0):
            continue
        if 'TowelFixedJawCollider' in contact['shape']:
            if abs(contact['surface_local_m'][0]+.0057) > 1e-6:
                continue
            jaw = 'fixed'
        elif 'moving_jaw_link/' in contact['shape']:
            jaw = 'moving'
        else:
            continue
        layers[half].add(particle); jaws.add(jaw)
    return {'candidate_present': bool(all(layers.values()) and jaws == {'fixed', 'moving'}),
            'layers': {h: sorted(ids) for h, ids in layers.items()},
            'jaws': sorted(jaws), 'four_sheet_grasp_validated': False,
            'scope': 'Initial immutable material witness for diagnostic lift only; does not prove layer-to-layer support or force closure.'}


def witness_motion(capture_local, current_local, capture_world, current_world, layers, limit_m=.003):
    if set(layers) != {0, 1} or not all(layers.values()):
        raise ValueError('both original material halves are required')
    slip = {h: float(np.max(np.linalg.norm(current_local[ids]-capture_local[ids], axis=1)))
            for h, ids in layers.items()}
    lift = {h: float(np.median(current_world[ids, 2]-capture_world[ids, 2])) for h, ids in layers.items()}
    return {'within_motion_bound': all(np.isfinite(v) and v <= limit_m for v in slip.values()),
            'maximum_original_material_motion_m': slip, 'median_lift_m': lift,
            'minimum_lift_reached': all(np.isfinite(v) and v >= .005 for v in lift.values())}
