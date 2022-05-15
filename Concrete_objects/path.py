from abc import ABC
from Concrete_objects import WayPoint


class Path(ABC):
    def __init__(self, path: list[WayPoint]):
        self.path = path
