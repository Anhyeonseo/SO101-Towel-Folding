"""Apply the existing immutable-material contact checks independently per arm.

Only the label namespace is normalized for the legacy left-arm functions.
World/local coordinates, normals, particle identities and distances are copied
unchanged. Contacts from the other arm can never satisfy a grasp gate.
"""
def arm_contacts(records, side):
    if side not in ('left', 'right'):
        raise ValueError('left or right arm required')
    return [dict(record, shape=record['shape'].replace('/right_', '/left_'))
            for record in records if f'/{side}_' in record['shape']]


def both_layers_lifted(capture, current, material_by_arm, minimum_m=.005):
    import numpy as np
    if set(material_by_arm) != {'left', 'right'}:
        raise ValueError('both immutable arm captures required')
    lifts = {}
    for side, layers in material_by_arm.items():
        if set(layers) != {0, 1} or not all(layers.values()):
            raise ValueError('both captured material layers required')
        lifts[side] = {str(h): float(np.median(current[ids, 2]-capture[side][ids, 2]))
                       for h, ids in layers.items()}
    return {'passed': all(value >= minimum_m for arm in lifts.values() for value in arm.values()),
            'layer_lift_m': lifts, 'minimum_layer_lift_m': minimum_m}


def freeze_layer_pairs(evidence):
    """Freeze the actual opposing point roles and material triangle per layer."""
    if not evidence['passed'] or set(evidence['layers']) != {'0', '1'}:
        raise ValueError('both qualified layers required')
    return {h: {'pair': list(evidence['layers'][str(h)]['opposing_contact_particle_pair']),
                'triangle': int(evidence['layers'][str(h)]['support_triangle_index'])}
            for h in (0, 1)}


def retained_layer_pairs(contacts, gate, frozen, *, friction_cone_check=True):
    """Require the SAME fixed/moving particles, triangle and friction geometry.

    A different particle on the original triangle cannot replace either role.
    Coordinates are never attached or reset by this observer. Disabling the
    cone check is for explicitly labelled diagnostic observation only.
    """
    layers = {}
    for h in (0, 1):
        fixed, moving = frozen[h]['pair']
        selected = [c for c in contacts if
                    (c['particle'] == fixed and 'TowelFixedJawCollider' in c['shape'])
                    or (c['particle'] == moving and 'moving_jaw_link/' in c['shape'])]
        layers[str(h)] = gate.evaluate(selected, 'left', .02,
            required_triangle_index=frozen[h]['triangle'], allow_same_particle=True,
            allow_outer_face_boundary=True, friction_cone_check=friction_cone_check)
    return {'passed': all(layer['passed'] for layer in layers.values()), 'layers': layers}


def frozen_pair_indices(frozen):
    return {h: list(dict.fromkeys(frozen[h]['pair'])) for h in (0, 1)}


def observed_material_bilateral_contacts(contacts, frozen):
    """Diagnostic contact presence only; not friction/force-closure validation.

    Keep original IDs, positive actual overlap and the registered outer pad
    surface. No opposing-angle or antipodal-line heuristic is applied here.
    All original material movement must still be checked by the caller.
    """
    import math
    layers = {}
    for h in (0, 1):
        selected = [c for c in contacts if c['particle'] in set(frozen[h]['pair'])]
        actual = [c for c in selected if math.isfinite(c['penetration_m']) and
                  math.isfinite(c['signed_distance_m']) and c['penetration_m'] > 0 and
                  c['signed_distance_m'] >= -1e-7]
        fixed = [c for c in actual if 'TowelFixedJawCollider' in c['shape'] and
                 abs(c['surface_local_m'][0]+.0057) <= 1e-6]
        moving = [c for c in actual if 'moving_jaw_link/' in c['shape']]
        layers[str(h)] = {'passed': bool(fixed and moving),
                          'original_triangle': frozen[h]['triangle'],
                          'fixed_particle_ids': sorted({c['particle'] for c in fixed}),
                          'moving_particle_ids': sorted({c['particle'] for c in moving}),
                          'actual_contacts': fixed+moving}
    return {'passed': all(v['passed'] for v in layers.values()), 'layers': layers,
            'diagnostic_only': True, 'friction_or_force_closure_validated': False}


def retained_material_patch(contacts, gate, frozen, *, minimum_overlap_m=None, friction_cone_check=True):
    """Allow jaw roles to redistribute ONLY within the original material IDs.

    Keep the original support triangle and default full friction-cone check.
    Disabling that check is only for explicitly labelled diagnostics.
    A third vertex, even on that triangle, cannot replace an original node.
    The caller still bounds every original node against its original position.
    """
    if minimum_overlap_m is not None:
        import math
        if not math.isfinite(minimum_overlap_m) or minimum_overlap_m <= 0:
            raise ValueError('positive finite contact reserve required')
        # Filter before selecting a qualified pair: the gate's ID ordering
        # otherwise lets a shallow contact hide an already sufficient pair.
        contacts = [c for c in contacts if c['penetration_m'] >= minimum_overlap_m]
    layers = {}
    for h in (0, 1):
        original_ids = set(frozen[h]['pair'])
        layers[str(h)] = gate.evaluate([c for c in contacts if c['particle'] in original_ids],
            'left', .02, required_triangle_index=frozen[h]['triangle'],
            allow_same_particle=True, allow_outer_face_boundary=True, friction_cone_check=friction_cone_check)
    return {'passed': all(layer['passed'] for layer in layers.values()), 'layers': layers}


def pair_contact_reserve(geometry, minimum_overlap_m):
    """Additional pre-lift gate on the already qualified immutable contacts.

    Overlap is the native particle-contact radius minus surface distance,
    not a calibrated rubber compression or grasp force.
    """
    import math
    if not math.isfinite(minimum_overlap_m) or minimum_overlap_m <= 0:
        raise ValueError('positive finite contact reserve required')
    overlaps = {}
    for h in ('0', '1'):
        layer = geometry.get('layers', {}).get(h, {})
        contacts = layer.get('actual_contacts', []) if layer.get('passed') else []
        values = [float(c['penetration_m']) for c in contacts]
        overlaps[h] = min(values) if len(values) >= 2 and all(math.isfinite(v) for v in values) else None
    return {'passed': bool(geometry['passed'] and all(
        v is not None and v >= minimum_overlap_m for v in overlaps.values())),
        'minimum_layer_overlap_m': overlaps, 'required_overlap_m': minimum_overlap_m}
