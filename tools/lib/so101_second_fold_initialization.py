"""Explicit diagnostic translation of a saved cloth shape, never a motion."""
import hashlib
import numpy as np


def diagnostic_initial_shape(source_nodes, x_shift_m=0.):
    nodes = np.asarray(source_nodes, dtype=np.float64)
    if nodes.ndim != 2 or nodes.shape[1] != 3 or not np.isfinite(nodes).all():
        raise ValueError('finite N by 3 source cloth shape required')
    if not np.isfinite(x_shift_m) or abs(x_shift_m) > .1:
        raise ValueError('diagnostic X translation must be finite and within 100 mm')
    result = nodes.copy()
    result[:, 0] += x_shift_m
    return result


def initial_shape_digest(nodes):
    return hashlib.sha256(np.asarray(nodes, dtype='<f8').tobytes()).hexdigest()
