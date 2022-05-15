from abc import ABC, abstractmethod
from Concrete_objects import MapfInstance, WayPoint, Path


class MAPFInput(ABC):
    def __init__(self, map_instance: MapfInstance, starts_list: list[WayPoint], goals_list: list[WayPoint]):
        self.map_instance = map_instance
        self.starts_list = starts_list
        self.goals_list = goals_list

    @abstractmethod
    def validate_input(self):
        raise NotImplementedError


class MAPFOutput(ABC):
    def __init__(self, paths: list[Path], cost, cpu_time):
        self.paths = paths  # paths[0] will be the path for agent 0
        self.cost = cost
        self.cpu_time = cpu_time


class MAPFAlgo(ABC):
    def __init__(self, attributes):
        pass

    @abstractmethod
    def solve(self, mapf_input: MAPFInput) -> MAPFOutput:
        raise NotImplementedError

