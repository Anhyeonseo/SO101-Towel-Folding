import numpy as np
from tools.lib.so101_contact_readback import soft_contact_geometry


def test_batch_geometry_matches_scalar_for_world_and_body_contacts():
    rng = np.random.default_rng(9)
    n = 1000
    transforms = rng.normal(size=(5, 7)).astype(np.float32)
    transforms[:, 3:] /= np.linalg.norm(transforms[:, 3:], axis=1)[:, None]
    shapes = rng.integers(0, 6, n)
    bodies = np.array([-1, 0, 1, 2, 3, 4])
    particles = np.arange(n)
    local = rng.normal(size=(n, 3)).astype(np.float32)
    normals = rng.normal(size=(n, 3)).astype(np.float32)
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    positions = rng.normal(size=(n, 3)).astype(np.float32)
    radii = np.full(n, .003, dtype=np.float32)
    expected = []
    for shape, v in zip(shapes, local):
        body = bodies[shape]
        if body >= 0:
            t = transforms[body]
            v = t[:3] + (v + 2 * np.cross(t[3:6], np.cross(t[3:6], v) + t[6] * v))
        expected.append(v)
    expected = np.asarray(expected)
    world, signed, penetration = soft_contact_geometry(
        shapes, particles, local, normals, positions, radii, bodies, transforms)
    np.testing.assert_allclose(world, expected, atol=1e-6)
    expected_signed = np.array([np.dot(n, p-v) for n, p, v in zip(normals, positions, expected)])
    np.testing.assert_allclose(signed, expected_signed, atol=2e-6)
    np.testing.assert_allclose(penetration, radii-expected_signed, atol=2e-6)
    np.testing.assert_array_equal(penetration > 0, radii-expected_signed > 0)


def test_empty_contact_buffer():
    result = soft_contact_geometry(np.array([], dtype=int), np.array([], dtype=int),
        np.empty((0, 3)), np.empty((0, 3)), np.empty((0, 3)), np.array([]),
        np.array([], dtype=int), np.empty((0, 7)))
    assert [a.shape for a in result] == [(0, 3), (0,), (0,)]
