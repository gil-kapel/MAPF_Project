from abc import ABC
from Concrete_objects import WayPoint


class Path(ABC):
    def __init__(self, path: list[WayPoint]):
        # !!!!! The time step of each location is it's index in the list !!!!!
        self.path = path

    def __len__(self):
        return len(self.path)
