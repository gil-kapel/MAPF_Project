from abc import ABC, abstractmethod
from . import MapfInstance, WayPoint


class MAPFInput(ABC):
    def __init__(self, map_instance: MapfInstance, starts_list: list[WayPoint], goals_list: list[WayPoint]):
        self.map_instance = map_instance
        self.starts_list = starts_list
        self.goals_list = goals_list

    @abstractmethod
    def validate_input(self):
        raise NotImplementedError

