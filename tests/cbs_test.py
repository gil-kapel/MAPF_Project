import sys
sys.path.append('./../')
import time as timer
from MAPFSolvers import CBSSolver, CBSInput
from Concrete_objects import FileMapInstance

if __name__ == '__main__':
    start = timer.time()
    print('Start parsing')
    map_instance = FileMapInstance()
    starts_list, goals_list = map_instance.create_map('../instances/test_47.txt')

    cbs_solver = CBSSolver()
    cbs_input = CBSInput(map_instance, starts_list, goals_list)
    solution = cbs_solver.solve(cbs_input)  # MAPF output

    print(f'make span = {solution.make_span}\n'
          f'sum of costs = {solution.sum_of_costs}\n'
          f'CPU time = {solution.cpu_time}\n')
    for i, path in enumerate(solution.paths):
        print(f'The path of agent {i} is: \n{path}')
    pass
