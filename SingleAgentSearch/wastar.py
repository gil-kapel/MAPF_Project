from abc import ABC
from collections import defaultdict
import numpy as np
from heapq import heappush, heappop
from Abstract_objects import LowLevelSearch, LLSInput
from Concrete_objects import WayPoint, MapfInstance, Constraint, Path


class AStarInput(LLSInput):
    def __init__(self, map_instance: MapfInstance, start_loc: WayPoint, goal_loc: WayPoint, agent: int,
                 constraints: list[Constraint], w=1):
        super(AStarInput, self).__init__(map_instance, start_loc, goal_loc, agent)
        self.constraints = constraints
        self.w = w


class AStarNode(ABC):
    def __init__(self, waypoint: WayPoint, time_step: int, g_score=np.inf, h_score=np.inf):
        self.h_score = h_score
        self.g_score = g_score
        self.waypoint = waypoint
        self.time_step = time_step
        self.prev_state = None
        self.closed = False
        self.opened = False

    def __lt__(self, other):
        return self.h_score + self.g_score < other.h_score + other.g_score

    def __eq__(self, other):
        return self.waypoint == other.waypoint and self.time_step == other.time_step


class AStar(LowLevelSearch):
    def __init__(self):
        super().__init__(True)
        self.constraints = None
        self.goal = None
        self.map_instance = None
        self.agent = None
        self.start_loc = None

    # search an optimal path for a specific agent
    def search(self, a_star_input: AStarInput):
        self.constraints = a_star_input.constraints
        self.goal = a_star_input.goal_loc
        self.start_loc = a_star_input.start_loc
        self.agent = a_star_input.agent
        self.map_instance = a_star_input.map_instance.map
        open_list = []
        closed_list = dict()
        h_value = self.compute_heuristics()[self.start_loc]
        c_table = self.build_constraint_table()
        root = {'loc': self.start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None}
        self.push_node(open_list, root)
        closed_list[(root['loc'])] = root
        while len(open_list) > 0:
            curr = self.pop_node(open_list)
            #############################
            # Task 1.4: Adjust the goal test condition to handle goal constraints
            if curr['loc'] == self.goal:
                if self.is_future_constraints_on_goal(curr['time_step'], c_table):
                    continue
                return self.get_path(curr)
            for direction in range(5):
                child_loc = self.move(curr['loc'], direction)
                if self.is_location_out_of_boundaries(child_loc) or \
                        self.map_instance[child_loc[0]][child_loc[1]] or \
                        self.is_constrained(curr['loc'], child_loc, curr['time_step'] + 1, c_table):
                    # prone if found a constraint
                    continue
                child = {'loc': child_loc,
                         'g_val': curr['g_val'] + 1,
                         'h_val': self.compute_heuristics()[child_loc],
                         'parent': curr,
                         'time_step': curr['time_step'] + 1}
                if (child['loc'], child['time_step']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['time_step'])]
                    if self.compare_nodes(child, existing_node):
                        closed_list[(child['loc']), child['time_step']] = child
                        self.push_node(open_list, child)
                else:
                    closed_list[(child['loc']), child['time_step']] = child
                    self.push_node(open_list, child)
            return None  # Failed to find solutions

    # Heuristic function for estimations
    def compute_heuristics(self):
        # Use Dijkstra to build a shortest-path tree rooted at the goal location
        open_list = []
        closed_list = dict()
        goal = self.goal
        my_map = self.map_instance
        root = {'loc': goal, 'cost': 0}
        heappush(open_list, (root['cost'], goal, root))
        closed_list[goal] = root
        while len(open_list) > 0:
            (cost, loc, curr) = heappop(open_list)
            for direction in range(4):
                child_loc = self.move(loc, direction)
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
                        heappush(open_list, (child_cost, child_loc, child))
                else:
                    closed_list[child_loc] = child
                    heappush(open_list, (child_cost, child_loc, child))
        # build the heuristics table
        h_values = dict()
        for loc, node in closed_list.items():
            h_values[loc] = node['cost']
        return h_values

    @staticmethod
    def move(loc, direction):
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
        return loc[0] + directions[direction][0], loc[1] + directions[direction][1]

    # Reached goal state and no constraints ahead
    def is_goal(self, state: AStarNode) -> bool:
        return state.waypoint == self.goal and state.time_step > self.goal

    # Reconstruct path
    @staticmethod
    def get_path(state: AStarNode):
        current = state
        path = []
        while current:
            path.append(current.waypoint)
            current = current.prev_state
        return Path(path)

    # make a constraint table as a dictionary from the AStar map
    def build_constraint_table(self) -> dict:
        time_step_dict = defaultdict(list)
        for constraint in self.constraints:
            if constraint.agent == self.agent:
                time_step_dict[constraint.time_step].append(constraint.position)
        return dict(time_step_dict)

    @staticmethod
    def push_node(open_list, node):
        heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

    @staticmethod
    def pop_node(open_list):
        _, _, _, curr = heappop(open_list)
        return curr

    @staticmethod
    def compare_nodes(n1, n2):
        """Return true is n1 is better than n2."""
        return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

    def is_future_constraints_on_goal(self, time_step, constraint_table):
        for key in constraint_table.keys():
            if key > time_step and [self.goal] in constraint_table[key]:
                return True
        return False

    @staticmethod
    def is_constrained(curr_loc, next_loc, next_time, constraint_table: dict):
        if next_time not in constraint_table:
            return False
        for location in constraint_table[next_time]:
            if len(location) == 2:
                if location[0] == curr_loc and location[1] == next_loc or \
                        location[0] == next_loc and location[1] == curr_loc:
                    return True
            elif location[0] == next_loc:
                return True
        return False

    def is_location_out_of_boundaries(self, child_loc):
        if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(self.map_instance) or \
                child_loc[1] >= len(self.map_instance[0]):
            return True
        return False
