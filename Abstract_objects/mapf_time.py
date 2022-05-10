from abc import ABC, abstractmethod
from time import time


class MAPFTime(ABC):
    def __init__(self, time_step=0):
        self.time_step = time_step
        self.global_time = time()

    def __add__(self, time_step):
        return MAPFTime(self.time_step + time_step)


t = MAPFTime(1)
t2 = MAPFTime(12)
print(t +2)
