from abc import ABC
from Concrete_objects import Position
# python -m pip install --upgrade --force-reinstall numpy-quaternion
# import numpy as np
# import quaternion
from sympy.algebras import Quaternion
from functools import singledispatchmethod


class WayPoint(ABC):
    @singledispatchmethod
    def __init__(self, x=-1.0, y=0.0, z=0.0, real=0.0, i=0.0, j=0.0, k=0.0):
        self.position = Position(x, y, z)
        self.orientation = Quaternion(real, i, j, k)

    @singledispatchmethod
    def __init__(self, position: Position, orientation: Quaternion):
        self.position = position
        self.orientation = orientation

    def __eq__(self, other):
        return self.position == other.position

    def __add__(self, other):
        position = self.position + other.position
        orientation = self.orientation + other.orientation
        return WayPoint(position, orientation)
