from abc import ABC, abstractmethod
from Abstract_objects import Path, LLSInput


class LowLevelSearch(ABC):
    def __init__(self, attr): # initial attributes for each lls algorithm
        pass

    # Search an optimal path for a specific agent
    @abstractmethod
    def search(self, lls_input: LLSInput) -> Path:
        raise NotImplementedError

