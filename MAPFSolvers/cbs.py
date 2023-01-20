import math
import time as timer
from abc import ABC
import heapq
import copy
import numpy as np
from typing import List

from Abstract_objects import MAPFSolver, MAPFOutput, MAPFInput, MapfInstance, Path, WayPoint
from Concrete_objects import Constraint, Collision
from MAPF_exceptions import NoSolution, WrongInput, TimeLimit
from SingleAgentSearch import AStar


class CBSNode(ABC):
    def __init__(self, constraints: List[Constraint], paths: List[Path], sum_of_cost=0, make_span=0):
        self.paths = paths
        self.constraints = constraints
        self.collisions = []
        self.sum_of_costs = sum_of_cost
        self.make_span = make_span

    def __lt__(self, other, method='sum of costs') -> bool:
        scores = {'sum of costs': self.sum_of_costs < other.sum_of_costs, 'make span': self.make_span < other.make_span}
        return scores[method]

    def calc_sum_of_costs(self):
        self.sum_of_costs = 0
        for path in self.paths:
            if path is None:
                continue
            self.sum_of_costs += len(path) - 1

    def calc_make_span(self):
        self.make_span = 0
        max_time = -np.inf
        for path in self.paths:
            if path is None:
                continue
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
                if path1 == path2 or path1 is None or path2 is None:
                    continue
                collision = self.detect_collision(i, path1, j, path2)
                if collision is None:
                    continue
                if not self.is_collision_in_list(collision):
                    self.collisions.append(collision)
        return self.collisions

    def detect_collision(self, agent1: int, path1: Path, agent2: int, path2: Path):
        for t, loc in enumerate(path1):
            if loc == self.get_location(path2, t):
                collision = Collision(agent1, agent2, t, loc)
                return collision
            elif loc == self.get_location(path2, t + 1) and \
                    self.get_location(path1, t + 1) == self.get_location(path2, t):
                collision = Collision(agent1, agent2, t + 1, loc, self.get_location(path1, t + 1))
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
        if collision.is_edge_collision:
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

    def update_node(self):
        self.calc_make_span()
        self.calc_sum_of_costs()
        self.detect_collisions()


class CBSInput(MAPFInput):
    def __init__(self, map_instance: MapfInstance, starts_list: List[WayPoint], goals_list: List[WayPoint]):
        super().__init__(map_instance, starts_list, goals_list)
        self.num_of_agents = len(starts_list)

    def validate_input(self):
        if self.num_of_agents < 0 or not self.are_agents_in_legal_places():
            raise WrongInput('Illegal map')
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
        return False


class CBSSolver(MAPFSolver):
    """CBS high-level search."""
    def __init__(self, time_limit=math.inf, low_level_search=AStar()):
        super().__init__(time_limit)
        self.low_level_search = low_level_search
        self.open_list = []
        self.start_time = 0
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.heuristics = {}

    def solve(self, cbs_input: CBSInput) -> MAPFOutput:
        cbs_input.validate_input()
        self.start_time = timer.time()
        root = CBSNode([], [])
        for i in range(cbs_input.num_of_agents):  # Find initial path for each agent
            goal = cbs_input.goals_list[i]
            if goal not in self.heuristics.keys():
                self.build_heuristics_for_goal(cbs_input.goals_list[i], cbs_input.map_instance)
            lls_input = self.low_level_search.input_factory(cbs_input.map_instance, cbs_input.starts_list[i],
                                                            cbs_input.goals_list[i], 0,
                                                            i, root.constraints, self.heuristics[goal])
            path = self.low_level_search.search(lls_input)
            if path is None:
                raise NoSolution()
            root.paths.append(path)
        root.update_node()
        self.push_node(root)
        while len(self.open_list) > 0:
            smallest_node = self.__pop_node()

            if timer.time() - self.start_time > self.time_limit * 1.2:
                raise TimeLimit(
                    MAPFOutput(smallest_node.paths, smallest_node.sum_of_costs, smallest_node.make_span, self.CPU_time))

            if len(smallest_node.collisions) == 0:
                self.CPU_time = timer.time() - self.start_time
                # print(f'Number of expanded nodes: {self.num_of_expanded}')
                # print(f'Number of generated nodes: {self.num_of_generated}')
                return MAPFOutput(smallest_node.paths, smallest_node.sum_of_costs, smallest_node.make_span, self.CPU_time)
            collision = smallest_node.collisions.pop()
            constraints = smallest_node.standard_splitting(collision)
            for constraint in constraints:
                new_constraints = copy.deepcopy(smallest_node).constraints
                new_constraints.append(copy.deepcopy(constraint))
                new_node = CBSNode(new_constraints, copy.deepcopy(smallest_node.paths),
                                   smallest_node.sum_of_costs, smallest_node.make_span)
                agent = constraint.agent
                goal = cbs_input.goals_list[agent]
                lls_input = self.low_level_search.input_factory(cbs_input.map_instance,
                                                                cbs_input.starts_list[agent], goal,
                                                                0, agent, new_node.constraints,
                                                                self.heuristics[goal])
                path = self.low_level_search.search(lls_input)
                if path is not None:
                    new_node.paths[agent] = path
                    new_node.update_node()
                    self.push_node(new_node)
        raise NoSolution()

    def build_heuristics_for_goal(self, goal: WayPoint, map_instance: MapfInstance):
        self.heuristics[goal] = self.compute_heuristics(map_instance.map, (goal.position.x, goal.position.y))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node.sum_of_costs, node.make_span, len(node.collisions),
                                        self.num_of_generated, node))
        self.num_of_generated += 1

    def __pop_node(self) -> CBSNode:
        _, _, _, _, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    @staticmethod
    def compute_heuristics(my_map, goal):
        # Use Dijkstra to build a shortest-path tree rooted at the goal location
        def move(loc, dir):
            directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
            return loc[0] + directions[dir][0], loc[1] + directions[dir][1]
        open_list = []
        closed_list = dict()
        root = {'loc': goal, 'cost': 0}
        heapq.heappush(open_list, (root['cost'], goal, root))
        closed_list[goal] = root
        while len(open_list) > 0:
            (cost, loc, curr) = heapq.heappop(open_list)
            for dir in range(4):
                child_loc = move(loc, dir)
                child_cost = cost + 1
                if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                        or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                    continue
                if my_map[child_loc[0]][child_loc[1]]:
                    continue
                child = {'loc': child_loc, 'cost': child_cost}
                if child_loc in closed_list:
                    existing_node = closed_list[child_loc]
                    if existing_node['cost'] > child_cost:
                        closed_list[child_loc] = child
                        # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                        heapq.heappush(open_list, (child_cost, child_loc, child))
                else:
                    closed_list[child_loc] = child
                    heapq.heappush(open_list, (child_cost, child_loc, child))

        # build the heuristics table
        h_values = dict()
        for loc, node in closed_list.items():
            h_values[loc] = node['cost']
        return h_values
