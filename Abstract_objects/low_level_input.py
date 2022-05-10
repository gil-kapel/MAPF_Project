from abc import ABC, abstractmethod
from . import MapfInstance, WayPoint


class LLSInput(ABC):
    def __init__(self, map_instance: MapfInstance, start_loc: WayPoint, goal_loc: WayPoint, agent: int):
        self.map_instance = map_instance
        self.start_loc = start_loc
        self.goal_loc = goal_loc
        self.agent = agent

