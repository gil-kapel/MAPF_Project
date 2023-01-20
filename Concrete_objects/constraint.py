from abc import ABC
from Abstract_objects import WayPoint


class Constraint(ABC):
    def __init__(self, agent: int, time_step: int, waypoint: WayPoint, sec_vertex_pos=None):
        self.agent = agent
        self.time_step = time_step
        self.position = waypoint.position
        self.is_edge_constraint = False
        self.sec_vertex = sec_vertex_pos
        if sec_vertex_pos:
            self.is_edge_constraint = True
            self.sec_vertex = sec_vertex_pos.position

    def __str__(self):
        ret = str(f'agent: {self.agent}, time step: {self.time_step}, first vertex: {self.position}')
        if self.is_edge_constraint:
            ret = ret + str(f', second vertex: {self.sec_vertex}')
        return ret

    def __repr__(self):
        ret = str(f'agent: {self.agent}, time step:{self.time_step}, first vertex: {self.position}')
        if self.is_edge_constraint:
            ret = ret + str(f'second vertex: {self.sec_vertex}')
        return ret
