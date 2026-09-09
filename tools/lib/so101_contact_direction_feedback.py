"""Bounded local contact-plane proposals; never a replacement retention gate."""
import math
import numpy as np
from scipy.optimize import minimize


def contact_direction_score(contacts, frozen, translation=None):
    """Worst-layer best-pair cone utilization, with contact-distance penalties.

    Only original material IDs qualify. Surfaces are locally approximated by
    the observed tangent planes; native contacts must validate every response.
    """
    delta = np.zeros(3) if translation is None else np.asarray(translation, dtype=float)
    if delta.shape != (3,) or not np.isfinite(delta).all():
        raise ValueError('finite 3D translation required')
    scores = []
    for layer in (0, 1):
        ids = set(frozen[layer]['pair'])
        fixed = [c for c in contacts if c['particle'] in ids and
                 'TowelFixedJawCollider' in c['shape'] and c['normal_local'][0] >= .5 and
                 abs(c['surface_local_m'][0] + .0057) <= 1e-6]
        moving = [c for c in contacts if c['particle'] in ids and 'moving_jaw_link/' in c['shape']]
        values = []
        for a in fixed:
            for b in moving:
                projections, distances = [], []
                for c in (a, b):
                    n = np.asarray(c['normal_world']); point = np.asarray(c['particle_world_m'])
                    surface = np.asarray(c['surface_world_m'])
                    if not np.isfinite(np.r_[n, point, surface]).all():
                        raise ValueError('nonfinite contact geometry')
                    distance = float((point-surface-delta) @ n)
                    distances.append(distance); projections.append(point-distance*n)
                gap = projections[1]-projections[0]; length = float(np.linalg.norm(gap)); utilization = []
                for c, sign in ((a, 1), (b, -1)):
                    axial = float(sign * (gap @ c['normal_world']))
                    mu = c['effective_friction_coefficient']
                    if not math.isfinite(mu) or mu <= 0:
                        raise ValueError('positive finite friction required')
                    if axial <= .0001 or axial > a['radius_m']+b['radius_m']:
                        utilization.append(100.)
                    else:
                        utilization.append(math.sqrt(max(0., length*length-axial*axial))/(axial*mu))
                penalty = sum(max(0., .0005-v)*1e4 + max(0., v-.0028)*1e4 for v in distances)
                values.append(max(utilization)+penalty)
        scores.append(min(values, default=100.))
    return max(scores)


def propose_contact_translation(contacts, frozen, maximum_step_m=.0001):
    if not math.isfinite(maximum_step_m) or not 0 < maximum_step_m <= .0005:
        raise ValueError('translation trust region must be in (0, 0.5 mm]')
    before = contact_direction_score(contacts, frozen)
    if before >= 100:
        return {'accepted': False, 'reason': 'missing usable original contact pair', 'score_before': before}
    radius = maximum_step_m * 1000
    objective = lambda x: contact_direction_score(contacts, frozen, np.asarray(x)*.001)
    fit = minimize(objective, np.zeros(3), method='SLSQP', bounds=[(-radius, radius)]*3,
                   constraints=[{'type': 'ineq', 'fun': lambda x: radius*radius-x@x}],
                   options={'maxiter': 16, 'ftol': 1e-6, 'eps': 1e-4})
    delta = np.asarray(fit.x)*.001
    if not np.isfinite(delta).all():
        return {'accepted': False, 'reason': 'nonfinite optimizer proposal', 'score_before': before}
    # SLSQP can exceed its spherical constraint by nanometres. Project onto
    # the actual bound, then re-evaluate; never discard or enlarge that bound.
    delta *= min(1., maximum_step_m/max(np.linalg.norm(delta), 1e-12))
    after = contact_direction_score(contacts, frozen, delta)
    accepted = bool(np.isfinite(delta).all() and np.linalg.norm(delta) <= maximum_step_m+1e-9
                    and after < before-.005)
    return {'accepted': accepted, 'score_before': before, 'score_predicted': after,
            'translation_world_m': delta.tolist(), 'optimizer_success': bool(fit.success),
            'native_validation_required': True}
