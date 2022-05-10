from abc import ABC, abstractmethod
from . import MAPFOutput, MAPFInput


class MAPFAlgo(ABC):
    def __init__(self, mapf_input: MAPFInput):
        # M.a.p.f input must include: map, starts_list, goals_list, low_level_search algo
        self.map_instance = mapf_input.map_instance
        self.starts_list = mapf_input.starts_list
        self.goals_list = mapf_input.goals_list
        self.num_of_agents = len(mapf_input.starts_list)
        self.low_level_search = mapf_input.low_level_search
        self.lls_input = mapf_input.lls_input

        self.start_time = 0
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

    @abstractmethod
    def solve(self) -> MAPFOutput:
        raise NotImplementedError

