from abc import ABC, abstractmethod
from Concrete_objects import Path, WayPoint, MapfInstance


class LLSInput(ABC):
    def __init__(self, map_instance: MapfInstance, start_loc: WayPoint, goal_loc, agent: int):
        self.map_instance = map_instance
        self.start_loc = start_loc
        self.goal_loc = goal_loc
        self.agent = agent


class LowLevelSearch(ABC):
    def __init__(self, attr):  # initial attributes for each lls algorithm
        pass

    # Search an optimal path for a specific agent
    @abstractmethod
    def search(self, lls_input: LLSInput) -> Path:
        raise NotImplementedError

