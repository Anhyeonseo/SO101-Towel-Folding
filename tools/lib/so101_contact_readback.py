"""Batch the read-only geometry calculations in Newton contact diagnostics."""
import numpy as np


def active_contact_arrays(contacts, count):
    """Copy only populated GPU contact rows, never the full reserved capacity."""
    import warp as wp
    names = ('soft_contact_shape', 'soft_contact_particle',
             'soft_contact_body_pos', 'soft_contact_normal')
    arrays = [getattr(contacts, name) for name in names]
    if count < 0 or any(count > array.shape[0] for array in arrays):
        raise ValueError('Contact count exceeds the allocated contact buffer')
    # Slicing after .numpy() first transfers every unused reserved row to CPU.
    # A torch view lets us narrow on-device before the device-to-host copy.
    return tuple(wp.to_torch(array)[:count].cpu().numpy() for array in arrays)


def soft_contact_geometry(shape_indices, particle_indices, local_surfaces,
                          world_normals, particle_positions, particle_radii,
                          shape_bodies, body_transforms):
    body_indices = shape_bodies[shape_indices]
    surfaces = local_surfaces.copy()
    attached = body_indices >= 0
    transforms = body_transforms[body_indices[attached]]
    q = transforms[:, 3:6]
    v = surfaces[attached]
    surfaces[attached] = transforms[:, :3] + (v + 2 * np.cross(
        q, np.cross(q, v) + transforms[:, 6, None] * v))
    signed = np.sum(world_normals * (particle_positions[particle_indices] - surfaces), axis=1)
    penetration = particle_radii[particle_indices] - signed
    return surfaces, signed, penetration
