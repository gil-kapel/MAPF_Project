from abc import ABC, abstractmethod


class Planner(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def plan(self):
        pass


