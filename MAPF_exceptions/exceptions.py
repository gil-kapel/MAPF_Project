class NoSolution(Exception):
    def __init__(self) -> None:
        print('No solution')


class WrongInput(Exception):
    def __init__(self, message) -> None:
        print(message)


class NoLowLevelSearchInserted(Exception):
    def __init__(self) -> None:
        print('Need to initialize a concrete low level search')


class NoFileException(Exception):
    def __init__(self, message) -> None:
        print(message)


class TimeLimit(Exception):
    def __init__(self, mapf_output):
        print('reached time limit, the solution has collisions')
        self.output = mapf_output
