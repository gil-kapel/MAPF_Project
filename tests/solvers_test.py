import sys
sys.path.append('./../')
import time as timer
from MAPFSolvers import PBSSolver, PBSInput, CBSSolver, CBSInput, DictCBSSolver, PrioritizedPlanningSolver
from Concrete_objects import FileMapInstance
from tests.visualize import Animation
from MAPF_exceptions import NoSolution, WrongInput, NoFileException


def choose_solver():
    global solver, mapf_input
    solver_type = input('choose a solver from this list:\n'
                        '1)\'CBS\' \n'
                        '2)\'PBS\' \n'
                        '3)\'DictCBS\' \n'
                        '4)\'Prioritized\' \n'
                        '5)All\n')
    if solver_type == '1' or solver_type.lower() == 'cbs':
        solver = CBSSolver(time_limit=1000)
        mapf_input = CBSInput(map_instance, starts_list, goals_list)
        return solver, mapf_input
    elif solver_type == '2' or solver_type.lower() == 'pbs':
        solver = PBSSolver(time_limit=1000)
        mapf_input = PBSInput(map_instance, starts_list, goals_list)
        return solver, mapf_input
    elif solver_type == '3' or solver_type.lower() == 'dictcbs':
        solver = DictCBSSolver(time_limit=1000)
        mapf_input = CBSInput(map_instance, starts_list, goals_list)
        return solver, mapf_input
    elif solver_type == '4' or solver_type.lower() == 'prioritized':
        solver = PrioritizedPlanningSolver(time_limit=1000)
        mapf_input = CBSInput(map_instance, starts_list, goals_list)
        return solver, mapf_input
    elif solver_type == '5' or solver_type.lower() == 'all':

        solver = [CBSSolver(), PBSSolver(), DictCBSSolver(), PrioritizedPlanningSolver()]
        mapf_input = [CBSInput(map_instance, starts_list, goals_list),
                      PBSInput(map_instance, starts_list, goals_list),
                      CBSInput(map_instance, starts_list, goals_list),
                      CBSInput(map_instance, starts_list, goals_list)]
        return solver, mapf_input
    else:
        print('not valid mapf algo')
        return None, None


def main_testing():
    global map_instance, starts_list, goals_list, solver, mapf_input
    start = timer.time()
    map_instance = FileMapInstance()
    chosen_map = input('choose map number from this set [1, 49]: ')
    print('Start parsing')
    try:
        starts_list, goals_list = map_instance.create_map(f'../instances/test_{chosen_map}.txt')
        # starts_list, goals_list = map_instance.create_map(f'../instances/Boston_0_256.map')
    except NoFileException:
        return
    print('map is parsed')

    try:
        solver, mapf_input = choose_solver()
    except WrongInput:
        return
    if not solver:
        return

    if isinstance(solver, list):
        solutions = []
        for i in range(4):
            print(f'start solving with {solver[i]}')
            try:
                solutions.append(solver[i].solve(mapf_input[i]))
            except WrongInput:
                return
            except NoSolution:
                continue
            if not solver:
                return

        import pandas as pd
        from tabulate import tabulate

        data = {
            "sum of costs": [solution.sum_of_costs for solution in solutions],
            "make span cost": [solution.make_span for solution in solutions],
            "cpu time": [solution.cpu_time for solution in solutions]
        }
        df = pd.DataFrame(data, index=[f'{solv}' for solv in solver])
        print(tabulate(df, headers='keys', tablefmt='psql'))

    else:
        print(f'solving with {solver}')
        try:
            solution = solver.solve(mapf_input)
        except NoSolution:
            return
        except WrongInput:
            return

        print(f'make span = {solution.make_span}\n'
              f'sum of costs = {solution.sum_of_costs}\n'
              f'CPU time = {solution.cpu_time}\n')
        # for i, path in enumerate(solution.paths):
        #     print(f'The path of agent {i} is: \n{path}')
        starts = [(int(loc.position.x), int(loc.position.y)) for loc in starts_list]
        goals = [(int(loc.position.x), int(loc.position.y)) for loc in goals_list]
        paths = [[(int(loc.position.x), int(loc.position.y)) for loc in path] for path in solution.paths]
        animation = Animation(map_instance.map, starts, goals, paths)
        # animation.save("output.mp4", 1.0)
        animation.show()


if __name__ == '__main__':
    while True:
        main_testing()

