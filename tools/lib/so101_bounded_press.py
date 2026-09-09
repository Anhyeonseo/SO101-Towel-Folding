"""Bounded descent stop from jaw encoder deflection, not simulator contact IDs."""
import math


class JawDeflectionStop:
    def __init__(self, *, threshold_rad, dwell_s, maximum_rad):
        if not (0 < threshold_rad < maximum_rad and dwell_s > 0):
            raise ValueError('invalid descent stop limits')
        self.threshold = threshold_rad
        self.dwell = dwell_s
        self.maximum = maximum_rad
        self.since = None
        self.last_time = None
        self.stopped = False

    def observe(self, time_s, commanded, actual):
        if not all(math.isfinite(x) for x in (time_s, commanded, actual)):
            raise ValueError('nonfinite encoder observation')
        if self.last_time is not None and time_s <= self.last_time:
            raise ValueError('encoder timestamps must increase')
        self.last_time = time_s
        deflection = commanded-actual
        if abs(deflection) > self.maximum:
            raise RuntimeError('jaw deflection exceeds bounded descent guard')
        if deflection >= self.threshold:
            if self.since is None:
                self.since = time_s
            self.stopped |= time_s-self.since >= self.dwell-1e-9
        else:
            self.since = None
        return {'stop_descent': self.stopped, 'jaw_deflection_rad': deflection,
                'scope': 'encoder response only; cloth contact and grasp require independent validation'}
