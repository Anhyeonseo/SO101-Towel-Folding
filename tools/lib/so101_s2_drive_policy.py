"""Position commands with one initialization; never overwrite contact dynamics."""
import numpy as np


class PositionDrivePolicy:
    def __init__(self, joint_ids, initial, jaw_limits, dt, jaw_speed=2.0):
        self.joint_ids = list(joint_ids)
        self.previous = np.asarray(initial, dtype=float).copy()
        self.jaw_limits = np.asarray(jaw_limits, dtype=float)
        self.dt = dt
        self.jaw_speed = jaw_speed
        self.state_writes = 0
        self.commands = 0
        self._validate(self.previous)

    def _validate(self, values):
        if values.shape != (12,) or not np.isfinite(values).all():
            raise ValueError('expected 12 finite joint targets')
        jaws = values[[5, 11]]
        if np.any(jaws < self.jaw_limits[:, 0]) or np.any(jaws > self.jaw_limits[:, 1]):
            raise ValueError('jaw target outside model limits')

    def initialize(self, robot, positions, velocities):
        if self.state_writes:
            raise RuntimeError('joint initialization may only occur once')
        robot.write_joint_state_to_sim_index(position=positions, velocity=velocities,
                                             joint_ids=self.joint_ids)
        self.state_writes += 1

    def command(self, robot, positions, values):
        if self.state_writes != 1:
            raise RuntimeError('initialize before commanding')
        values = np.asarray(values, dtype=float)
        self._validate(values)
        if np.max(np.abs(values[[5, 11]]-self.previous[[5, 11]])) > self.jaw_speed*self.dt+1e-7:
            raise ValueError('jaw target velocity limit exceeded')
        robot.set_joint_position_target_index(target=positions, joint_ids=self.joint_ids)
        self.previous = values.copy()
        self.commands += 1
