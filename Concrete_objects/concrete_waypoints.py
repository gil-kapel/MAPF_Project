from Abstract_objects import WayPoint
# python -m pip install --upgrade --force-reinstall numpy-quaternion
# import numpy as np
# import quaternion
from sympy.algebras import Quaternion


class TimedWayPoint(WayPoint):
    def __init__(self, x=0, y=0, z=0, t=0):
        super().__init__(x, y, z)
        self.time_step = t

    def __eq__(self, other):
        return self.position == other.position and self.time_step == other.time_step

    def __add__(self, other):
        position = self.position + other.position
        time = self.time_step + other.time_step
        return TimedWayPoint(position.x, position.y, position.z, time)

    def __hash__(self):
        return hash((self.position.x, self.position.y, self.position.z, self.time_step))

    def __str__(self):
        return str(f'(Position: {self.position} in time step: {self.time_step})')

    def __repr__(self):
        return str(f'(Position: {self.position} in time step: {self.time_step})')


class SevenDimWayPoint(WayPoint):
    def __init__(self, x=0, y=0, z=0, real=0, i=0, j=0, k=0):
        super().__init__(x, y, z)
        self.orientation = Quaternion(real, i, j, k)

    def __eq__(self, other):
        return self.position == other.position

    def __add__(self, other):
        position = self.position + other.position
        orien = (self.orientation + other.orientation).args
        return SevenDimWayPoint(position.x, position.y, position.z, orien[0], orien[1], orien[2], orien[3])

    def __hash__(self):
        return hash((self.position.x, self.position.y, self.position.z))

    def __str__(self):
        return str(f'(Position: {self.position}, Orientation: {self.orientation})')

    def __repr__(self):
        return str(f'(Position: {self.position}, Orientation: {self.orientation})')
