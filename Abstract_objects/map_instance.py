from abc import ABC, abstractmethod


class MapfInstance(ABC):
    def __init__(self):
        self.map = []
        self.obstacles_inst = True
        self.blank_space = False

    @abstractmethod
    def create_map(self, **kwargs):
        raise NotImplementedError
