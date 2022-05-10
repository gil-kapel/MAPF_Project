from abc import ABC, abstractmethod
from collections import defaultdict
import numpy as np
from heapq import heappush, heappop
from Abstract_objects import WayPoint


class State(ABC):
    def __init__(self, waypoint: WayPoint, g_score=np.inf, h_score=np.inf):
        self.h_score = h_score
        self.g_score = g_score
        self.waypoint = waypoint
        self.prev_state = None
        self.closed = False
        self.opened = False

    def __lt__(self, other):
        return self.h_score + self.g_score < other.h_score + other.g_score

    def __eq__(self, other):
        return self.waypoint == other.waypoint

    def is_constrained(self, next_loc, next_time, constraint_table: dict):
        if next_time not in constraint_table:
            return False
        for location in constraint_table[next_time]:
            if len(location) == 2:
                if location[0] == self.waypoint and location[1] == next_loc or \
                        location[0] == next_loc and location[1] == self.waypoint:
                    return True
            elif location[0] == next_loc:
                return True
        return False


class StateDict(dict):
    def __missing__(self, waypoint):
        state = State(waypoint)
        self.__setitem__(waypoint, state)
        return state


class LowLevelSearch(ABC):
    def __init__(self, map_instance, start_loc, goal_loc, agent, constraints=None):
        self.states = StateDict()
        self.open_states = []
        self.closed_states = []
        self.map_instance = map_instance
        self.start = State(start_loc, g_score=0)
        self.goal = State(goal_loc)
        self.agent = agent
        self.constraints = constraints
        heappush(self.open_states, self.start)

    # Heuristic function for estimations
    @abstractmethod
    def heuristic_func(self, state_a: State, state_b: State):
        raise NotImplementedError

    # Distance measure between two states
    @abstractmethod
    def distance(self, a, b):
        raise NotImplementedError

    # Get neighbors based on agent available steps
    @abstractmethod
    def get_neighbors(self, state):
        raise NotImplementedError

    # Reached goal state and no constraints ahead
    def is_goal(self, state: State) -> bool:
        return state == self.goal and state.waypoint.time_step > self.goal.waypoint.time_step

    # Reconstruct path
    @staticmethod
    def get_path(state: State):
        current = state
        while current:
            yield current.waypoint
            current = current.prev_state

    def build_constraint_table(self) -> dict:
        time_step_dict = defaultdict(list)
        for constraint in self.constraints:
            if constraint.agent == self.agent:
                time_step_dict[constraint.time_step].append(constraint.position)
        return dict(time_step_dict)

    # search an optimal path for a specific agent
    @abstractmethod
    def search(self, max_iterations=int(2e5), debug=False):
        raise NotImplementedError

