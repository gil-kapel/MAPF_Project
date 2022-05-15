from abc import ABC
from Concrete_objects import Position
# python -m pip install --upgrade --force-reinstall numpy-quaternion
# import numpy as np
# import quaternion
from sympy.algebras import Quaternion


class WayPoint(ABC):
    def __init__(self, x=-1.0, y=0.0, z=0.0, real=0.0, i=0.0, j=0.0, k=0.0, time_step=0):
        self.position = Position(x, y, z)
        self.orientation = Quaternion(real, i, j, k)
