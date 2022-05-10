from abc import ABC, abstractmethod
import numpy as np
from heapq import heappush, heappop
from Abstract_objects import WayPoint, Path, LLSInput


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


class StateDict(dict):
    def __missing__(self, waypoint):
        state = State(waypoint)
        self.__setitem__(waypoint, state)
        return state


class LowLevelSearch(ABC):
    def __init__(self):
        # self.states = StateDict()
        # self.open_states = []
        # self.closed_states = []
        # self.map_instance = lls_input.map_instance
        # self.start = State(lls_input.start_loc, g_score=0)
        # self.goal = State(lls_input.goal_loc)
        # self.agent = lls_input.agent
        # heappush(self.open_states, self.start)
        pass

    # Search an optimal path for a specific agent
    @abstractmethod
    def search(self, lls_input: LLSInput) -> Path:
        raise NotImplementedError

