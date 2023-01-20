from abc import ABC
from Abstract_objects import WayPoint


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
        if self.is_edge_collision:
            return self.agent1 == other.agent1 and self.agent2 == other.agent2 and self.time_step == other.time_step \
                   and self.position == other.position and self.sec_vertex_pos == other.sec_vertex_pos
        else:
            return self.agent1 == other.agent1 and self.agent2 == other.agent2 and self.time_step == other.time_step \
                   and self.position == other.position

    def __str__(self):
        ret = str(f'agent1: {self.agent1}, agent2: {self.agent2}, time step: {self.time_step}, '
                  f'first vertex: {self.position.position}')
        if self.is_edge_collision:
            ret = ret + str(f', second vertex: {self.sec_vertex_pos.position}')
        return ret

    def __repr__(self):
        ret = str(f'agent1: {self.agent1}, agent2: {self.agent2}, time step: {self.time_step}, '
                  f'first vertex: {self.position.position}')
        if self.is_edge_collision:
            ret = ret + str(f', second vertex: {self.sec_vertex_pos.position}')
        return ret
