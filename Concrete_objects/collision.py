from abc import ABC
from Concrete_objects import WayPoint


class Collision(ABC):
    def __init__(self, agent1: int, agent2: int, time_step: int,  position: WayPoint, sec_vertex_pos=None):
        self.agent1 = agent1
        self.agent2 = agent2
        self.time_step = time_step
        self.position = position
        self.sec_vertex_pos = sec_vertex_pos
        self.is_edge_collision = False
        if sec_vertex_pos:
            self.is_edge_collision = True

    def __eq__(self, other):
        return self.agent1 == other.agent1 and self.agent2 == other.agent2 and self.time_step == other.time_step \
               and self.position == other.position and self.sec_vertex_pos == other.sec_vertex_pos