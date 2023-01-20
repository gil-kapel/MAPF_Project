from abc import ABC
from typing import List
from Abstract_objects import WayPoint


class Path(ABC):
    def __new__(cls, path: List[WayPoint]):
        return path

    @property
    def path(self):
        return self.path

    def __len__(self):
        return len(self.path)

    def __iter__(self):
        return self

    def __str__(self):
        return str(f'{self.path}')

    def __repr__(self):
        return str(f'{self.path}')
