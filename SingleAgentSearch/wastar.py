from abc import ABC
import numpy as np
from heapq import heappush, heappop
from typing import List
from Abstract_objects import LowLevelSearch, LLSInput, WayPoint, MapfInstance, Path
from Concrete_objects import Constraint
from MAPF_exceptions import WrongInput


class AStarInput(LLSInput):
    def __init__(self, map_instance: MapfInstance, start_loc: WayPoint, goal_loc: WayPoint, goal_time: int, agent: int,
                 constraints: List[Constraint], heuristics):
        super(AStarInput, self).__init__(map_instance, start_loc, goal_loc, goal_time, agent)
        self.constraints = constraints
        self.heuristics = heuristics


class AStarNode(ABC):
    def __init__(self, position, time_step: int, g_score=np.inf, h_score=np.inf, prev_state=None):
        self.h_score = h_score
        self.g_score = g_score
        self.position = position
        self.time_step = time_step
        self.prev_state = prev_state
        self.closed = False
        self.opened = False

    def __lt__(self, other):
        return self.h_score + self.g_score < other.h_score + other.g_score

    def __eq__(self, other):
        return self.position == other.position and self.time_step == other.time_step


class AStar(LowLevelSearch):
    def __init__(self, w=0.5):
        super().__init__(w)
        self.w = w
        self.constraints = None
        self.goal = None
        self.goal_time = None
        self.map_instance = None
        self.agent = None
        self.start_loc = None

    # search an optimal path for a specific agent
    def search(self, a_star_input: AStarInput):
        self.constraints = a_star_input.constraints
        self.goal = a_star_input.goal_loc
        self.goal_time = a_star_input.goal_time
        self.start_loc = a_star_input.start_loc
        self.agent = a_star_input.agent
        self.map_instance = a_star_input.map_instance.map
        open_list = []
        closed_list = dict()
        root_loc = self.start_loc.position.x, self.start_loc.position.y
        try:
            h_value = a_star_input.heuristics[root_loc]
        except KeyError:
            raise WrongInput('\n\n---at least one of the agents can\'t arrive to his goal location---\n\n')
        c_table = self.build_constraint_table()
        root = AStarNode(root_loc, 0, 0, h_value, None)
        self.push_node(open_list, root)
        closed_list[(root_loc, root.time_step)] = root
        while len(open_list) > 0:
            curr = self.__pop_node(open_list)
            if curr.position == (self.goal.position.x, self.goal.position.y) and \
                    not self.is_future_constraints_on_goal(curr.time_step, c_table):
                return self.get_path(curr)
            for direction in range(5):
                child_loc = self.move(curr.position[0], curr.position[1], direction)
                if self.is_location_out_of_boundaries(child_loc) or \
                        self.map_instance[child_loc[0]][child_loc[1]] or \
                        self.is_constrained(curr.position, child_loc, curr.time_step + 1, c_table):
                    # prone if found a constraint
                    continue
                child = AStarNode(child_loc, curr.time_step + 1, curr.g_score + 1,
                                  a_star_input.heuristics[child_loc], curr)
                child_loc = child.position[0], child.position[1]
                if (child_loc, child.time_step) in closed_list:
                    existing_node = closed_list[(child_loc, child.time_step)]
                    if self.compare_nodes(child, existing_node):
                        closed_list[(child_loc, child.time_step)] = child
                        self.push_node(open_list, child)
                else:
                    closed_list[(child_loc, child.time_step)] = child
                    self.push_node(open_list, child)
        return None  # Failed to find solutions

    def input_factory(self, map_instance: MapfInstance, start_loc: WayPoint, goal_loc: WayPoint, goal_time: int,
                      agent: int, constraints: List[Constraint], heuristics):
        return AStarInput(map_instance, start_loc, goal_loc, goal_time, agent, constraints, heuristics)

    @staticmethod
    def l2(curr_point, goal_point):
        return np.sqrt((int(curr_point[0]) - int(goal_point[0])) ** 2 + (int(curr_point[1]) - int(goal_point[1])) ** 2)

    # Heuristic function for estimations
    def compute_heuristics(self, curr_point):
        return self.l2(curr_point, (self.goal.position.x, self.goal.position.y))

    # 5 options for an agent movement from his current location, each vertical direction and stay at place
    # only two dimensions (x, y)
    @staticmethod
    def move(x, y, direction) -> (int, int):
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
        x += directions[direction][0]
        y += directions[direction][1]
        return x, y

    # Reached goal state
    def is_goal(self, state: AStarNode) -> bool:
        return state.position == self.goal.position and self.goal_time == 0 or state.time_step <= self.goal_time

    # Reconstruct path
    @staticmethod
    def get_path(state: AStarNode):
        current = state
        path = []
        while current:
            path.append(WayPoint(current.position[0], current.position[1]))
            current = current.prev_state
        path.reverse()
        return Path(path)

    # make a constraint table as a dictionary from the AStar map
    def build_constraint_table(self) -> dict:
        time_step_dict = {}
        for constraint in self.constraints:
            if constraint.agent == self.agent:
                time = constraint.time_step
                if time in time_step_dict.keys():
                    time_step_dict[time].append(constraint)
                else:
                    time_step_dict[time] = [constraint]
        return time_step_dict

    def push_node(self, node_list, node: AStarNode):
        heappush(node_list, (self.w * node.g_score + (1 - self.w) * node.h_score, node.h_score, node.position, node))

    @staticmethod
    def __pop_node(node_list) -> AStarNode:
        _, _, _, curr = heappop(node_list)
        return curr

    def compare_nodes(self, n1: AStarNode, n2: AStarNode):
        """Return true is n1 is better than n2."""
        return self.w * n1.g_score + (1 - self.w) * n1.h_score < self.w * n2.g_score + (1 - self.w) * n2.h_score

    def is_future_constraints_on_goal(self, time_step, constraint_table):
        for key in constraint_table.keys():
            if key > time_step:
                for constraint in constraint_table[key]:
                    if constraint.is_edge_constraint:
                        if constraint.position == self.goal.position and constraint.sec_vertex == self.goal.position:
                            return True
                    elif constraint.position == self.goal.position:
                        return True
        return False

    def is_goal_in_constraint_table(self, constraints):
        for constraint in constraints:
            if self.goal.position == constraint.position:
                return True
        return False

    @staticmethod
    def is_constrained(curr_loc, next_loc, next_time, constraint_table: dict):
        if next_time not in constraint_table:
            return False
        constraints = constraint_table[next_time]
        for constraint in constraints:
            if constraint.is_edge_constraint:
                first_vertex = constraint.position.x, constraint.position.y
                second_vertex = constraint.sec_vertex.x, constraint.sec_vertex.y
                if first_vertex == curr_loc and second_vertex == next_loc or \
                        first_vertex == next_loc and second_vertex == curr_loc:
                    return True
            elif (constraint.position.x, constraint.position.y) == next_loc:
                return True
        return False

    def is_location_out_of_boundaries(self, child_loc):
        if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(self.map_instance) or \
                child_loc[1] >= len(self.map_instance[0]):
            return True
        return False
