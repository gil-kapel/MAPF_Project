from abc import ABC, abstractmethod
from . import MAPFOutput, MAPFInput


class MAPFAlgo(ABC):
    def __init__(self, attributes):
        pass

    @abstractmethod
    def solve(self, mapf_input: MAPFInput) -> MAPFOutput:
        raise NotImplementedError

