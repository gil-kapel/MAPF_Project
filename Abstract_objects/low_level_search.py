from abc import ABC, abstractmethod
from Concrete_objects import Path


class LLSInput(ABC):
    def __init__(self):
        pass


class LowLevelSearch(ABC):
    def __init__(self, attr):  # initial attributes for each lls algorithm
        pass

    # Search an optimal path for a specific agent
    @abstractmethod
    def search(self, lls_input: LLSInput) -> Path:
        raise NotImplementedError

