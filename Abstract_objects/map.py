from abc import ABC, abstractmethod
from . import WayPoint
import numpy as np
obstacles_inst = -1
blank_space = 0
agents_position_inst = 1


class MapfInstance(ABC):
    # TODO add comments - why did i implemented it? / what is the pourpse
    def __init__(self, vertical_size: int, horizon_size: int, agents_pos: list[WayPoint], obstacles: list[WayPoint]):
        self.map = np.array(vertical_size * horizon_size)
        self.x_size = vertical_size
        self.y_size = horizon_size
        self.obstacles = obstacles
        self.agents_pos = agents_pos
        for x in range(vertical_size):
            for y in range(horizon_size):
                point = WayPoint(x, y)
                if point in obstacles:
                    self.map[x][y] = obstacles_inst
                elif point in agents_pos:
                    self.map[x][y] = agents_position_inst
                else:
                    self.map[x][y] = blank_space
