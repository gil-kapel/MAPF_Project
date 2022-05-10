from abc import ABC, abstractmethod

from low_level_search import LowLevelSearch
from . import MapfInstance, LLSInput


class MAPFInput(ABC):
    def __init__(self, map_instance: MapfInstance, starts_list, goals_list):
        self.map_instance = map_instance
        self.starts_list = starts_list
        self.goals_list = goals_list

    @abstractmethod
    def validate_input(self):
        raise NotImplementedError

