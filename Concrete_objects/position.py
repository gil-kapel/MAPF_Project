from abc import ABC


class Position(ABC):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Position(self.x + other.x, self.y + other.y, self.z + other.z)

    def __eq__(self, other):
        return self.x, self.y, self.z == other.x, other.y, other.z
