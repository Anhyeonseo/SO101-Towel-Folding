"""Simulation-only STL closure guard and same-triangle contact qualification.

Uses the real outer pad face and solver contact points/normals. No inferred
6 mm moving-jaw plane, and no allowance for rigid pad interpenetration.
"""
from __future__ import annotations
from collections import defaultdict
import math
import numpy as np


class MeshPinchError(RuntimeError):
    pass


def clipped_minimum_x(triangles, footprint_equations):
    """Exact minimum X where triangles intersect a convex YZ footprint prism.

    Clip boundary triangles too: checking only their vertices misses crossings
    when all three vertices lie outside the pad outline.
    """
    triangles = np.asarray(triangles, dtype=float)
    equations = np.asarray(footprint_equations, dtype=float)
    distances = triangles[:, :, 1:] @ equations[:, :2].T + equations[:, 2]
    outside = np.any(np.all(distances > 1e-12, axis=1), axis=1)
    inside = np.all(distances <= 1e-12, axis=(1, 2))
    minimum = float(triangles[inside, :, 0].min()) if inside.any() else math.inf
    for triangle in triangles[~outside & ~inside]:
        if triangle[:, 0].min() >= minimum:
            continue
        polygon = list(triangle)
        for equation in equations:
            if not polygon:
                break
            clipped = []
            previous = polygon[-1]
            previous_d = previous[1:] @ equation[:2] + equation[2]
            for point in polygon:
                distance = point[1:] @ equation[:2] + equation[2]
                if (distance <= 0) != (previous_d <= 0):
                    clipped.append(previous + previous_d / (previous_d-distance)*(point-previous))
                if distance <= 0:
                    clipped.append(point)
                previous, previous_d = point, distance
            polygon = clipped
        if polygon:
            minimum = min(minimum, min(point[0] for point in polygon))
    return minimum


class MeshClosureGuard:
    """Conservative no-crossing guard in the common gripper-link frame."""
    def __init__(self, pad, moving_triangles, joint_rotation, joint_translation, axis):
        from scipy.spatial import ConvexHull
        self.pad = pad
        vertices = np.asarray(pad['vertices_m'])
        hull = ConvexHull(np.unique(vertices[:, 1:], axis=0))
        if not np.isclose(hull.volume, pad['source_patch_area_m2'], rtol=1e-6):
            raise MeshPinchError('pad footprint is not convex; cannot use prism guard')
        self.equations = hull.equations
        self.bounds = [vertices[:, 1:].min(0), vertices[:, 1:].max(0)]
        self.triangles = np.asarray(moving_triangles)
        self.rotation = np.asarray(joint_rotation)
        self.translation = np.asarray(joint_translation)
        self.axis = np.asarray(axis, dtype=float)

    def clearance(self, angle):
        from scipy.spatial.transform import Rotation
        rotation = self.rotation @ Rotation.from_rotvec(self.axis * angle).as_matrix()
        triangles = self.triangles @ rotation.T + self.translation
        mask = (np.all(triangles.max(1)[:, 1:] >= self.bounds[0], axis=1)
                & np.all(triangles.min(1)[:, 1:] <= self.bounds[1], axis=1))
        return clipped_minimum_x(triangles[mask], self.equations) - self.pad['outer_x_m']

    def derive_stop(self, requested_angle, open_angle, margin_m=.00025):
        from scipy.optimize import brentq
        if not all(math.isfinite(v) for v in (requested_angle, open_angle, margin_m)) or margin_m <= 0 or requested_angle > open_angle:
            raise MeshPinchError('invalid ordered closure interval or clearance margin')
        if self.clearance(open_angle) < margin_m:
            raise MeshPinchError('open jaw already violates pad clearance')
        stop = (requested_angle if self.clearance(requested_angle) >= margin_m else
                brentq(lambda q: self.clearance(q)-margin_m, requested_angle, open_angle, xtol=1e-10))
        samples = [self.clearance(q) for q in np.linspace(stop, open_angle, 41)]
        if min(samples) < margin_m-1e-8:
            raise MeshPinchError('closure has an interior unsafe branch')
        return {'minimum_model_angle_rad': float(stop), 'margin_m': margin_m,
                'minimum_sampled_clearance_m': min(samples), 'closure_samples': 41,
                'requested_model_angle_rad': requested_angle, 'open_model_angle_rad': open_angle,
                'scope': 'triangle-prism nonpenetration guard, not rubber-force validation'}


class ActualMeshPinchGate:
    def __init__(self, triangles, *, grid_side=None, required_material_axis=None):
        self.triangles = np.asarray(triangles, dtype=int)
        if self.triangles.ndim != 2 or self.triangles.shape[1] != 3:
            raise MeshPinchError('actual cloth triangle topology required')
        if required_material_axis not in (None, "column"):
            raise MeshPinchError("unsupported material closing direction")
        if required_material_axis is not None and (not isinstance(grid_side, int) or grid_side < 2):
            raise MeshPinchError("material closing direction requires the cloth grid side")
        self.grid_side = grid_side
        self.required_material_axis = required_material_axis
        self.by_particle = defaultdict(set)
        for index, triangle in enumerate(self.triangles):
            for particle in triangle:
                self.by_particle[int(particle)].add(index)

    def evaluate(self, contacts, side, maximum_pair_distance_m, *, required_triangle_index=None,
                 allow_same_particle=False, allow_outer_face_boundary=False,
                 friction_cone_check=False):
        if required_triangle_index is not None and not 0 <= required_triangle_index < len(self.triangles):
            raise MeshPinchError("required contact triangle is outside the cloth topology")
        fixed, moving = [], []
        for contact in contacts:
            if f'/{side}_' not in contact['shape']:
                continue
            if not all(np.all(np.isfinite(contact[key])) for key in
                       ['surface_world_m', 'particle_world_m', 'normal_world', 'normal_local', 'surface_local_m']):
                raise MeshPinchError('nonfinite solver contact geometry')
            if any(not np.isclose(np.linalg.norm(contact[key]), 1.0, atol=1e-4) for key in ('normal_world', 'normal_local')):
                raise MeshPinchError('solver contact normals must have unit length')
            if not all(math.isfinite(contact[key]) for key in ('penetration_m', 'signed_distance_m', 'radius_m')) or contact['radius_m'] <= 0:
                raise MeshPinchError('invalid solver contact distances')
            if not (contact['penetration_m'] > 0 and contact['signed_distance_m'] >= -1e-7):
                continue  # Speculative contacts and centres inside a rigid surface do not prove a pinch.
            if 'TowelFixedJawCollider' in contact['shape']:
                # At a mesh perimeter Newton uses particle-to-closest-point,
                # not the triangle's planar normal. A native boundary pinch
                # requires an inward component >=50% of the contact normal,
                # plus the opposing-jaw normal test below (within 60 degrees
                # of exact opposition). Do not require a planar-face normal
                # while the particle wraps around the outer-face perimeter.
                # The point must still be on the OUTER plane, not rim/back.
                minimum_alignment = .5 if allow_outer_face_boundary else .999
                if (contact['normal_local'][0] >= minimum_alignment
                        and abs(contact['surface_local_m'][0] - (-.0057)) <= 1e-6):
                    fixed.append(contact)  # Actual outer face only, never its rim/back.
            elif 'moving_jaw_link/' in contact['shape'] and 'moving_jaw_so101' in contact['shape']:
                moving.append(contact)
        candidates = []
        for f in fixed:
            for m in moving:
                fi, mi = int(f['particle']), int(m['particle'])
                if fi == mi and not allow_same_particle:
                    continue
                if fi == mi and not np.allclose(f['particle_world_m'], m['particle_world_m'], atol=1e-7, rtol=0):
                    raise MeshPinchError('one particle has inconsistent contact positions')
                if self.required_material_axis == "column" and fi % self.grid_side == mi % self.grid_side:
                    # Lateral neighbours do not establish a pucker across the
                    # closing direction of the upright, world-X-aligned jaws.
                    continue
                shared = self.by_particle[fi] & self.by_particle[mi]
                if required_triangle_index is not None:
                    shared &= {required_triangle_index}
                if not shared:
                    continue
                nf, nm = np.array(f['normal_world']), np.array(m['normal_world'])
                nf /= np.linalg.norm(nf); nm /= np.linalg.norm(nm)
                if not friction_cone_check and nf @ nm > -.5:
                    continue  # Both forces must oppose within 60 degrees on a curved jaw.
                gap = np.array(m['surface_world_m'])-f['surface_world_m']
                fixed_gap, moving_gap = float(gap @ nf), float(-gap @ nm)
                if min(fixed_gap, moving_gap) <= .0001:
                    continue
                if max(fixed_gap, moving_gap) > f['radius_m']+m['radius_m']:
                    continue
                if friction_cone_check:
                    # Antipodal friction geometry: the line joining actual
                    # surfaces must lie inside both solver friction cones.
                    # A fixed angle between curved face normals is insufficient.
                    length = float(np.linalg.norm(gap))
                    cone_passed = True
                    for c, axial in ((f, fixed_gap), (m, moving_gap)):
                        mu = c['effective_friction_coefficient']
                        if not math.isfinite(mu) or mu < 0:
                            raise MeshPinchError('invalid actual solver friction coefficient')
                        tangent = math.sqrt(max(0., length*length-axial*axial))
                        cone_passed &= tangent <= mu*axial + 1e-9
                    if not cone_passed:
                        continue
                pair_distance = float(np.linalg.norm(np.array(f['particle_world_m'])-m['particle_world_m']))
                if pair_distance > maximum_pair_distance_m:
                    continue
                triangle_index = min(shared)
                candidates.append((pair_distance, triangle_index, fi, mi, f, m, fixed_gap, moving_gap))
        if not candidates:
            return {'passed': False, 'reason': 'no qualified same-triangle opposing outer-face/STL contacts',
                    'eligible_fixed_contacts': len(fixed), 'eligible_moving_contacts': len(moving)}
        _, triangle_index, fi, mi, f, m, fixed_gap, moving_gap = min(candidates, key=lambda v:v[:4])
        support = self.triangles[triangle_index].tolist()
        return {'passed': True, 'gate': ('actual_opposing_contacts_on_material_patch' if allow_same_particle
                                        else 'actual_solver_outer_pad_and_curved_stl_same_triangle'),
                'unique_contact_particle_count': len({fi, mi}),
                'selected_distinct_particles': list(dict.fromkeys((fi, mi))),
                'opposing_contact_particle_pair': [fi, mi],
                'finite_element_support_vertex_indices': support,
                'support_triangle_index': triangle_index,
                # Solver forces act at these cloth nodes. Their exact triangle
                # barycentric coordinates preserve the existing release scoring.
                'anchors': [{'triangle_indices': support,
                             'barycentric_weights': [float(i == particle) for i in support]}
                            for particle in (fi, mi)],
                'actual_contacts': [f, m], 'opposing_projected_gap_m': [fixed_gap, moving_gap],
                'fixed_inward_normal_component': float(f['normal_local'][0]),
                'opposing_normal_dot': float(np.dot(f['normal_world'], m['normal_world'])),
                'retention_model': ('observed native solver contacts; no cloth node constraint'
                                    if allow_same_particle else
                                    'no-slip finite-element support; not a friction or force validation')}


def load_closure_guard(pad, urdf_path, root, side):
    """Read both joint and collision origins from the exact imported URDF."""
    import hashlib
    from pathlib import Path
    import xml.etree.ElementTree as ET
    import trimesh
    from scipy.spatial.transform import Rotation
    urdf_path = Path(urdf_path)
    digest = hashlib.sha256(urdf_path.read_bytes()).hexdigest()
    if digest != pad['source_urdf_sha256']:
        raise MeshPinchError('closure guard URDF differs from pad registration')
    robot = ET.parse(urdf_path).getroot()
    joint = robot.find(f"joint[@name='{side}_gripper_joint']")
    collisions = robot.findall(f"link[@name='{side}_moving_jaw_link']/collision")
    if joint is None or len(collisions) != 1:
        raise MeshPinchError('expected one registered moving-jaw collision')
    collision = collisions[0]
    mesh = collision.find('geometry/mesh')
    prefix = 'package://so101_description/'
    if mesh is None or not mesh.get('filename', '').startswith(prefix):
        raise MeshPinchError('unrecognized moving jaw mesh')
    mesh_path = Path(root)/'ros2_ws/src/so101_description'/mesh.get('filename')[len(prefix):]
    triangles = np.array(trimesh.load_mesh(mesh_path).triangles)
    triangles *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
    def origin(element):
        o = element.find('origin')
        if o is None:
            return np.eye(3), np.zeros(3)
        return (Rotation.from_euler('xyz', np.fromstring(o.get('rpy', '0 0 0'), sep=' ')).as_matrix(),
                np.fromstring(o.get('xyz', '0 0 0'), sep=' '))
    rotation, translation = origin(collision)
    triangles = triangles @ rotation.T + translation
    rotation, translation = origin(joint)
    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
    if not np.isclose(np.linalg.norm(axis), 1):
        raise MeshPinchError('joint axis is not unit length')
    return MeshClosureGuard(pad, triangles, rotation, translation, axis), {
        'urdf_sha256': digest, 'moving_mesh_sha256': hashlib.sha256(mesh_path.read_bytes()).hexdigest()}


class PersistentMeshClosure:
    """Hold on current contact; resume bounded closure if that contact is lost.

    Completion requires the same actual cloth triangle throughout the hold,
    independently for each jaw. This never reuses first-touch success.
    """
    def __init__(self, open_angles, hard_limits, closing_steps, hold_steps):
        self.targets = dict(open_angles)
        self.limits = dict(hard_limits)
        if closing_steps < 1 or hold_steps < 1 or set(self.targets) != set(self.limits):
            raise MeshPinchError('invalid persistent closure configuration')
        self.increments = {s: (q-self.limits[s])/closing_steps for s,q in self.targets.items()}
        if any(not math.isfinite(v) or v <= 0 for v in self.increments.values()):
            raise MeshPinchError('closure requires finite ordered jaw limits')
        self.hold_steps = hold_steps
        self.counts = {s: 0 for s in self.targets}
        self.triangles = {s: None for s in self.targets}
        self.losses = {s: 0 for s in self.targets}

    def observe(self, gates):
        for side in self.targets:
            gate = gates[side]
            if gate['passed']:
                triangle = tuple(sorted(gate['finite_element_support_vertex_indices']))
                self.counts[side] = self.counts[side]+1 if self.triangles[side] == triangle else 1
                self.triangles[side] = triangle
            else:
                if self.counts[side]:
                    self.losses[side] += 1
                self.counts[side] = 0
                self.triangles[side] = None
                self.targets[side] = max(self.limits[side], self.targets[side]-self.increments[side])
        return all(n >= self.hold_steps for n in self.counts.values())


def fixed_pad_last_release_order(evidence):
    """Release uncontacted support, moving contact, then stationary pad contact.

    Contact roles, not vertex numbers or tied one-hot weights, determine which
    support remains constrained while the moving jaw opens.
    """
    if evidence.get('passed') is not True:
        raise MeshPinchError('release requires a qualified actual-surface pinch')
    support = list(evidence['finite_element_support_vertex_indices'])
    fixed, moving = evidence['selected_distinct_particles']
    if len(set(support)) != 3 or fixed == moving or not {fixed, moving} <= set(support):
        raise MeshPinchError('release requires distinct fixed/moving nodes in one triangle')
    return [i for i in support if i not in (fixed, moving)] + [moving, fixed]
