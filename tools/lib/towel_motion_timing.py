"""Time parametrization preserving an existing piecewise-linear joint path."""
import numpy as np


def smooth_path_samples(waypoints, duration_s, dt_s):
    points = np.asarray(waypoints, dtype=float)
    if points.ndim != 2 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('finite joint waypoints required')
    if not np.isfinite(duration_s) or not np.isfinite(dt_s) or min(duration_s,dt_s)<=0:
        raise ValueError('positive finite timing required')
    steps = max(2, int(np.ceil(duration_s / dt_s)))
    t = np.arange(1,steps+1) / steps
    progress = (10*t**3-15*t**4+6*t**5)*(len(points)-1)
    index = np.minimum(np.floor(progress).astype(int),len(points)-2)
    fraction = progress-index
    return points[index]+fraction[:,None]*(points[index+1]-points[index])
