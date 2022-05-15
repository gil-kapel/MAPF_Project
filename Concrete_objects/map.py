from abc import ABC
from Concrete_objects import WayPoint
import numpy as np


class MapfInstance(ABC):
    # TODO add comments - why did i implemented it? / what is the purpose
    def __init__(self, vertical_size: int, horizon_size: int, obstacles: list[WayPoint]):
        self.map = np.matrix(vertical_size, horizon_size)
        self.x_size = vertical_size
        self.y_size = horizon_size
        self.obstacles = obstacles
        self.obstacles_inst = -1
        self.blank_space = 0
        self.agents_position_inst = 1
        for x in range(vertical_size):
            for y in range(horizon_size):
                point = WayPoint(x, y)
                if point in obstacles:
                    self.map[x][y] = self.obstacles_inst
                else:
                    self.map[x][y] = self.blank_space
