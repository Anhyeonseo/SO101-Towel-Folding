"""Decimate viewport updates while preserving every physics step."""

from functools import wraps
import time


def pace_live_rendering(step, physics_dt, *, maximum_frame_gap=.05, clock=time.monotonic):
    """Catch up physics between GUI frames instead of waiting for 240 redraws/s.

    Every caller-requested physics step still executes. Wall-time pacing only
    chooses when to draw. The frame-gap limit keeps the GUI responsive if the
    physics itself cannot keep up with real time.
    """
    if physics_dt <= 0 or maximum_frame_gap <= 0:
        raise ValueError('Physics dt and frame gap must be positive')
    origin = None
    simulated = 0.0
    last_render = None

    @wraps(step)
    def paced(render=True):
        nonlocal origin, simulated, last_render
        now = clock()
        if origin is None:
            origin = now
            last_render = now
        simulated += physics_dt
        draw = bool(render and (simulated >= now-origin or now-last_render >= maximum_frame_gap))
        result = step(render=draw)
        finished = clock()
        if draw:
            last_render = finished
        # Initialization or a paused GUI must not create a catch-up burst.
        if finished-now > .5:
            origin = finished-simulated
        return result

    return paced


def decimate_rendering(step, interval: int):
    """Wrap SimulationContext.step; explicit render=False always wins.

    Isaac Lab 3's step() does not consume SimulationCfg.render_interval.
    Keep this counter separate from solver timing and direct render() calls.
    """
    if isinstance(interval, bool) or not isinstance(interval, int) or interval < 1:
        raise ValueError("render interval must be a positive integer")
    count = 0

    @wraps(step)
    def step_with_render_interval(render=True):
        nonlocal count
        count += 1
        return step(render=bool(render and count % interval == 0))

    return step_with_render_interval
