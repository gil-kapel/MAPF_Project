import time as timer
from Abstract_objects import MAPFSolver, MAPFOutput, MAPFInput, MapfInstance, Path, WayPoint
from Concrete_objects import Constraint, Collision
from MAPF_exceptions import NoSolution, WrongInput, NoLowLevelSearchInserted
from .cbs import CBSSolver, CBSInput, CBSNode


class PrioritizedPlanningSolver(CBSSolver):
    """A planner that plans for each robot sequentially."""

    def solve(self, cbs_input: CBSInput) -> MAPFOutput:
        """ Finds paths for all agents from their start locations to their goal locations."""
        self.start_time = timer.time()
        result = []
        constraints = []
        cbs_input.validate_input()
        num_of_agents = len(cbs_input.starts_list)
        for i in range(num_of_agents):  # Find path for each agent
            if i not in self.heuristics.keys():
                self.build_heuristics_for_goal(cbs_input.goals_list[i], cbs_input.map_instance)
            lls_input = self.low_level_search.input_factory(cbs_input.map_instance, cbs_input.starts_list[i],
                                                            cbs_input.goals_list[i], 0,
                                                            i, constraints, self.heuristics[cbs_input.goals_list[i]])
            path = self.low_level_search.search(lls_input)
            if path is None:
                raise NoSolution()
            result.append(path)
            for t, loc in enumerate(path):
                for agent in range(i+1, num_of_agents):
                    constraints.append(Constraint(agent, t, loc))
                    if t > 0:
                        constraints.append(Constraint(agent, t, loc, path[t-1]))
                    if t == len(path) - 1:
                        board_area = len(cbs_input.map_instance.map) ** 2
                        for j in range(board_area):
                            constraints.append(Constraint(agent, t + j, loc))
        node = CBSNode([], result)
        self.CPU_time = timer.time() - self.start_time
        node.calc_sum_of_costs()
        node.calc_make_span()
        return MAPFOutput(result, node.sum_of_costs, node.make_span, self.CPU_time)
