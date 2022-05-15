import time as timer
from abc import ABC
import heapq
from Abstract_objects import MAPFAlgo, MAPFOutput, MAPFInput, LLSInput
from Concrete_objects import WayPoint, MapfInstance, Path, Constraint, Collision

import copy
import numpy as np
from MAPF_exceptions import NoSolution


class HighLevelNode(ABC):
    def __init__(self, constraints: list[Constraint], paths: list[Path]):
        self.paths = paths
        self.constraints = constraints
        self.collisions = []
        self.sum_of_costs = 0
        self.make_span = 0

    def __lt__(self, other, method='sum of costs') -> bool:
        scores = {'sum of costs': self.sum_of_costs < other.sum_of_costs, 'make span': self.make_span < other.make_span}
        return scores[method]

    def sum_of_costs_calc(self):
        for path in self.paths:
            self.sum_of_costs += len(path.path) - 1

    def make_span_calc(self):
        max_time = -np.inf
        for path in self.paths:
            if len(path.path) > max_time:
                max_time = len(path.path)
        self.make_span = max_time

    def make_constraints(self, disjoint=False):
        self.detect_collisions()
        if disjoint:
            self.constraints = self.disjoint_splitting(self.collisions)
        else:
            self.constraints = self.standard_splitting(self.collisions)

    def detect_collisions(self):
        for i, path1 in enumerate(self.paths):
            for j, path2 in enumerate(self.paths):
                if path1 == path2:
                    continue
                res = self.detect_collision(path1, path2)
                if res is None:
                    continue
                collision = Collision(i, j, res.time_step, res.position, res.sec_vertex_pos)
                if not self.is_collision_in_list(collision):
                    self.collisions.append(collision)
        return self.collisions

    def detect_collision(self, path1: Path, path2: Path):
        for t, loc in enumerate(path1.path):
            if self.get_location(path1.path, t) == self.get_location(path2.path, t):
                collision = Collision(path1.agent, path2.agent, t, self.get_location(path1.path, t))
                return collision
            elif self.get_location(path1.path, t) == self.get_location(path2.path, t + 1) and \
                    self.get_location(path1.path, t + 1) == self.get_location(path2.path, t):
                collision = Collision(path1.agent, path2.agent, t + 1, self.get_location(path1.path, t),
                                      self.get_location(path1.path, t + 1))
                return collision
        return None

    @staticmethod
    def get_location(path, time):
        if time < 0:
            return path[0]
        elif time < len(path):
            return path[time]
        else:
            return path[-1]  # wait at the goal location

    def is_collision_in_list(self, collision: Collision) -> bool:
        reverse_collision = copy.deepcopy(collision)
        reverse_collision.agent1 = collision.agent2
        reverse_collision.agent2 = collision.agent1
        if not collision.is_edge_collision:
            reverse_collision.position = collision.sec_vertex_pos
            reverse_collision.sec_vertex_pos = collision.position
        if collision in self.collisions or reverse_collision in self.collisions:
            return True
        return False

    @staticmethod
    def standard_splitting(collision):
        location = collision['loc']
        time_step = collision['timestep']
        a1 = collision['a1']
        a2 = collision['a2']
        constraints = []
        if len(location) == 1:  # vertex constraint
            constraints.append({'agent': a1, 'loc': location, 'timestep': time_step})
            constraints.append({'agent': a2, 'loc': location, 'timestep': time_step})
        else:  # edge constraint
            constraints.append({'agent': a1, 'loc': location, 'timestep': time_step})
            constraints.append({'agent': a2, 'loc': [location[1], location[0]], 'timestep': time_step})
        return constraints

    @staticmethod
    def disjoint_splitting(collision):
        # TODO disjoint splitting cbs
        constraints = []
        return constraints


class CBSInput(MAPFInput):
    def __init__(self, map_instance: MapfInstance, starts_list, goals_list):
        super().__init__(map_instance, starts_list, goals_list)

    def validate_input(self):
        pass
        # TODO


class CBSSolver(MAPFAlgo):
    """CBS high-level search."""
    def __init__(self, attributes):
        # M.a.p.f input must include: map, starts_list, goals_list, low_level_search algo
        self.low_level_search = attributes.low_level_search
        self.open_list = []
        self.start_time = 0
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        super().__init__(attributes)

    def push_node(self, node):
        heapq.heappush(self.open_list, node)
        self.num_of_generated += 1

    def pop_node(self):
        node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def solve(self) -> MAPFOutput:
        self.start_time = timer.time()
        root = HighLevelNode([], [])
        for i in range(self.num_of_agents):  # Find initial path for each agent
            # TODO 2: think how to make the lls abstract - here the LLS Factory will choose the algo (A*..)
            #  and the next line needs to do the job
            # LLS Input needs to be according to the low level search
            lls = self.low_level_search.search(LLSInput(self.map_instance, self.starts_list[i], self.goals_list[i], i,
                                                        root.constraints))
            path = lls.search()
            if path is None:
                raise NoSolution()
            root.paths.append(path)
        root.sum_of_costs_calc()
        root.make_span_calc()
        root.make_constraints()
        self.push_node(root)
        while len(self.open_list) > 0:
            smallest_node = self.pop_node()
            if len(smallest_node['collisions']) == 0:
                self.CPU_time = timer.time() - self.start_time
                return MAPFOutput(smallest_node.paths, smallest_node.sum_of_costs, self.CPU_time)
            for constraint in smallest_node.constraints:
                new_constraints = copy.deepcopy(smallest_node).constraints
                new_constraints.append(constraint)
                new_node = HighLevelNode(new_constraints, copy.deepcopy(smallest_node.paths))
                agent = constraint.agent
                path = self.low_level_search(self.map_instance, self.starts_list[agent], self.goals_list[agent],
                                             agent, new_constraints).search()
                if path is not None:
                    new_node.paths[agent] = path
                    new_node.detect_collisions()
                    new_node.sum_of_costs_calc()
                    self.push_node(new_node)
        raise NoSolution()
