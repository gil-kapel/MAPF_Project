import math
from abc import ABC, abstractmethod
from typing import List
from Abstract_objects import MapfInstance, WayPoint, Path


class MAPFInput(ABC):
    def __init__(self, map_instance: MapfInstance, starts_list: List[WayPoint], goals_list: List[WayPoint]):
        self.map_instance = map_instance
        self.starts_list = starts_list
        self.goals_list = goals_list

    @abstractmethod
    def validate_input(self):
        raise NotImplementedError


class MAPFOutput(ABC):
    def __init__(self, paths: List[Path], sum_of_costs=0, make_span=0, cpu_time=0.0):
        self.paths = paths  # paths[0] will be the path for agent 0
        self.sum_of_costs = sum_of_costs
        self.make_span = make_span
        self.cpu_time = cpu_time


class MAPFSolver(ABC):
    def __init__(self, time_limit):
        self.time_limit = time_limit

    @abstractmethod
    def solve(self, mapf_input: MAPFInput) -> MAPFOutput:
        raise NotImplementedError

    def __str__(self):
        return str(f'{self.__class__.__name__}')

    def __repr__(self):
        return str(f'{self.__class__.__name__}')
