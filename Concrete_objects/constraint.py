from abc import ABC
from Concrete_objects import WayPoint


class Constraint(ABC):
    def __init__(self, agent: int, time_step: int, position: WayPoint, sec_vertex_pos=None):
        self.agent = agent
        self.time_step = time_step
        self.position = position
        self.is_edge_constraint = False
        if sec_vertex_pos:
            self.is_edge_constraint = True
