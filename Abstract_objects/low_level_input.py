from abc import ABC, abstractmethod


class LLSInput(ABC):
    def __init__(self, map_instance, start_loc, goal_loc, agent):
        self.map_instance = map_instance
        self.start_loc = start_loc
        self.goal_loc = goal_loc
        self.agent = agent

