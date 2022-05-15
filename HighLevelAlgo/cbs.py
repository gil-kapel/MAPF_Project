import time as timer
from abc import ABC
import heapq
import random
import copy
import numpy as np
from Abstract_objects import MAPFAlgo, MAPFOutput, MAPFInput
from Concrete_objects import MapfInstance, Path, Constraint, Collision, WayPoint
from MAPF_exceptions import NoSolution, WrongInput


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
            self.sum_of_costs += len(path) - 1

    def make_span_calc(self):
        max_time = -np.inf
        for path in self.paths:
            if len(path) > max_time:
                max_time = len(path)
        self.make_span = max_time

    def make_constraints(self, disjoint=False):
        self.detect_collisions()
        if disjoint:
            self.constraints = self.disjoint_splitting(self.collisions[0])
        else:
            self.constraints = self.standard_splitting(self.collisions[0])

    def detect_collisions(self):
        for i, path1 in enumerate(self.paths):
            for j, path2 in enumerate(self.paths):
                if path1 == path2:
                    continue
                res = self.detect_collision(i, path1, j, path2)
                if res is None:
                    continue
                collision = Collision(i, j, res.time_step, res.position, res.sec_vertex_pos)
                if not self.is_collision_in_list(collision):
                    self.collisions.append(collision)
        return self.collisions

    def detect_collision(self, agent1: int, path1: Path, agent2: int, path2: Path):
        for t, loc in enumerate(path1.path):
            if self.get_location(path1.path, t) == self.get_location(path2.path, t):
                collision = Collision(agent1, agent2, t, self.get_location(path1.path, t))
                return collision
            elif self.get_location(path1.path, t) == self.get_location(path2.path, t + 1) and \
                    self.get_location(path1.path, t + 1) == self.get_location(path2.path, t):
                collision = Collision(agent1, agent2, t + 1, self.get_location(path1.path, t),
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
    def standard_splitting(collision: Collision):
        location = collision.position
        time_step = collision.time_step
        a1 = collision.agent1
        a2 = collision.agent2
        constraints = []
        if not collision.is_edge_collision:  # vertex constraint
            constraints.append(Constraint(a1, time_step, location))
            constraints.append(Constraint(a2, time_step, location))
        else:  # edge constraint
            constraints.append(Constraint(a1, time_step, location, collision.sec_vertex_pos))
            constraints.append(Constraint(a2, time_step, collision.sec_vertex_pos, location))
        return constraints

    @staticmethod
    def disjoint_splitting(collision: Collision):
        return []
        # disjoint_constraints = []
        # positive_agent = random.randint(0, 1)
        # if positive_agent == 0:
        #     agent1 = collision.agent1
        #     agent2 = collision.agent2
        # else:
        #     agent2 = collision.agent1
        #     agent1 = collision.agent2
        # loc = collision.position
        # if not collision.is_edge_collision: # TODO got to a goal
        #     collision1 = Collision(agent1, agent2, collision.time_step, loc, True)
        #     collision2 = {'agent': , 'loc': loc, 'time_step': collision['time_step'], 'positive': False}
        # elif len(loc) == 1 and collision['goal'] == 1:
        #     collision1 = {'agent': collision['a1'], 'loc': loc, 'time_step': collision['time_step'], 'positive': False}
        #     collision2 = {'agent': collision['a2'], 'loc': loc, 'time_step': collision['time_step'], 'positive': True}
        # else:
        #     collision1 = {'agent': agent1, 'loc': [loc[0], loc[1]], 'time_step': collision['time_step'], 'positive': True}
        #     collision2 = {'agent': agent2, 'loc': [loc[1], loc[0]], 'time_step': collision['time_step'],
        #                   'positive': False}
        # disjoint_constraints.append(collision1)
        # disjoint_constraints.append(collision2)
        # print(disjoint_constraints)
        # return disjoint_constraints


class CBSInput(MAPFInput):
    def __init__(self, map_instance: MapfInstance, starts_list: list[WayPoint], goals_list: list[(WayPoint, int)]):
        super().__init__(map_instance, starts_list, goals_list)
        self.num_of_agents = len(starts_list)

    def validate_input(self):
        if self.num_of_agents < 0 or self.are_agents_in_legal_places():
            raise WrongInput()
        if self.start_points_collisions() or self.goal_points_collision():
            raise NoSolution()

    def are_agents_in_legal_places(self):
        for point in self.starts_list:
            try:
                if self.map_instance.map[point.position.x][point.position.y] == self.map_instance.obstacles_inst:
                    return False
            except ValueError:
                return False
        for point in self.goals_list:
            try:
                if self.map_instance.map[point.position.x][point.position.y] == self.map_instance.obstacles_inst:
                    return False
            except ValueError:
                return False
        return True

    def start_points_collisions(self):
        if len(self.starts_list) != len(set(self.starts_list)):
            return True
        else:
            return False

    def goal_points_collision(self):
        if len(self.goals_list) != len(set(self.goals_list)):
            return True
        else:
            return False


class CBSSolver(MAPFAlgo):
    """CBS high-level search."""
    def __init__(self, attributes):
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

    def solve(self, cbs_input: CBSInput) -> MAPFOutput:
        cbs_input.validate_input()
        self.start_time = timer.time()
        root = HighLevelNode([], [])
        for i in range(cbs_input.num_of_agents):  # Find initial path for each agent
            lls = self.low_level_search.search(cbs_input.map_instance, cbs_input.starts_list[i],
                                               cbs_input.goals_list[i], i, root.constraints)
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
                path = self.low_level_search(cbs_input.map_instance, cbs_input.starts_list[agent],
                                             cbs_input.goals_list[agent], agent, new_constraints).search()
                if path is not None:
                    new_node.paths[agent] = path
                    new_node.detect_collisions()
                    new_node.sum_of_costs_calc()
                    self.push_node(new_node)
        raise NoSolution()
