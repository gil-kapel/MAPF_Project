from abc import ABC, abstractmethod
from . import Path


class MAPFOutput(ABC):
    def __init__(self, paths: list[Path], cost, cpu_time):
        self.paths = paths
        self.cost = cost
        self.cpu_time = cpu_time
