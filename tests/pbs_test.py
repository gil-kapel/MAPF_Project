import sys
sys.path.append('./../')
import time as timer
from MAPFSolvers import PBSSolver, PBSInput
from Concrete_objects import FileMapInstance
from MAPF_exceptions import NoSolution, WrongInput, NoFileException
from tests.visualize import Animation

if __name__ == '__main__':
    start = timer.time()
    pbs_solver = PBSSolver()

    map_path = input("Enter the full path for your map: ")
    map_instance = FileMapInstance()

    try:
        starts_list, goals_list = map_instance.create_map(map_path)

    except NoFileException:
        starts_list, goals_list = [], []
        exit(0)

    try:
        pbs_input = PBSInput(map_instance, starts_list, goals_list)

    except WrongInput:
        pbs_input = None
        exit(0)
    try:
        start = timer.time()
        print('Start solving first time..')
        solution = pbs_solver.solve(pbs_input)
        print(f'first time = {timer.time() - start}')
        pbs_input = PBSInput(map_instance, starts_list, goals_list, solution.priority_order)
        start = timer.time()
        print('Start solving second time..')
        solution = pbs_solver.solve(pbs_input)
        print(f'second time = {timer.time() - start}\n'
              f'priority order = {solution.priority_order}')
    except NoSolution:
        solution = None
        exit(0)

    # print(f'make span = {solution.make_span}\n'
    #       f'sum of costs = {solution.sum_of_costs}\n'
    #       f'CPU time = {solution.cpu_time}\n')
    # for i, path in enumerate(solution.paths):
    #     print(f'The path of agent {i} is: \n{path}')
    starts = [(int(loc.position.x), int(loc.position.y)) for loc in starts_list]
    goals = [(int(loc.position.x), int(loc.position.y)) for loc in goals_list]
    paths = [[(int(loc.position.x), int(loc.position.y)) for loc in path] for path in solution.paths]
    animation = Animation(map_instance.map, starts, goals, paths)
    # animation.save("output.mp4", 1.0)
    animation.show()
