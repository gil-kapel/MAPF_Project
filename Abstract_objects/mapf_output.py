from abc import ABC, abstractmethod
from . import Path


class MAPFOutput(ABC):
    def __init__(self, paths: list[Path], cost, cpu_time):
        self.paths = paths  # paths[0] will be the path for agent 0
        self.cost = cost
        self.cpu_time = cpu_time
