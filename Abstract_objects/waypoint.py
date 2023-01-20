from abc import ABC
from sympy import sympify
from sympy.core.expr import Expr


class WayPoint(ABC):
    def __init__(self, x=0, y=0, z=0):
        self.position = Position(x, y, z)

    def __eq__(self, other):
        return self.position == other.position

    def __add__(self, other):
        position = self.position + other.position
        return WayPoint(position.x, position.y, position.z)

    def __hash__(self):
        return hash(self.position)

    def __str__(self):
        return str(f'(Position: {self.position})')

    def __repr__(self):
        return str(f'(Position: {self.position})')


class Position(Expr):
    def __new__(cls, x=0.0, y=0.0, z=0.0):
        x = sympify(x)
        y = sympify(y)
        z = sympify(z)
        obj = Expr.__new__(cls, x, y, z)
        obj._x = x
        obj._y = y
        obj._z = z
        return obj

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    def __add__(self, other):
        return Position(self.x + other.x, self.y + other.y, self.z + other.z)

    def __radd__(self, other):
        return Position(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Position(self.x - other.x, self.y - other.y, self.z - other.z)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __neg__(self):
        return Position(-self.x, -self.y, -self.z)

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __str__(self):
        return str(f'({self.x}, {self.y}, {self.z})')

    def __repr__(self):
        return str(f'({self.x}, {self.y}, {self.z})')
