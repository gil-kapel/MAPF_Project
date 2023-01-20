import time as timer
from collections import defaultdict
import heapq
import copy
from typing import List
from Abstract_objects import MAPFOutput, MapfInstance, Path, WayPoint
from Concrete_objects import Constraint
from MAPF_exceptions import NoSolution, WrongInput, TimeLimit
from .cbs import CBSSolver, CBSInput, CBSNode


class PBSNode(CBSNode):
    def __init__(self, constraints: List[Constraint], paths: List[Path], priority_order: List, number_of_agents,
                 sum_of_cost=0, make_span=0):
        # PBS must have a path list to work with if an initial ordering is given
        if len(paths) == 0:
            paths = [None] * number_of_agents
        super().__init__(constraints, paths, sum_of_cost, make_span)
        self.number_of_agents = number_of_agents
        self.priority_order = priority_order

    def create_prior_agent_constraints(self, prior_agents, agent):
        """Create constraints for an agent by considering only agents that are prior then him"""
        constraints = []
        for prior_agent in prior_agents:
            for t, loc in enumerate(self.paths[prior_agent]):
                constraints.append(Constraint(agent, t, loc))
                if t > 0:
                    constraints.append(Constraint(agent, t, loc, self.paths[prior_agent][t - 1]))
                if t == len(self.paths[prior_agent]) - 1:
                    # Add constraints to inferior agents on the prior's goal location for reasonable time
                    self.calc_sum_of_costs()
                    board_area = self.sum_of_costs * 2
                    for j in range(board_area):
                        constraints.append(Constraint(agent, t + j, loc))
        return constraints

    def topological_sort_util(self, start, edges, visited, sort):
        """Perform topological sort on a directed acyclic graph."""
        current = start
        visited.append(current)
        neighbors = edges[current]
        for neighbor in neighbors:
            if neighbor not in visited:
                sort = self.topological_sort_util(neighbor, edges, visited, sort)
        sort.append(current)
        return sort

    def topological_sort(self, agent):
        """Create two queues for prior and inferior agents then 'agent'"""
        edges, reverse_edges = self.create_graph()

        # Consider 'agent' as a inferior agent for later usage - therefore he won't be at prior queue
        inferior_queue = self.topological_sort_util(agent, edges, [], [])
        prior_queue = self.topological_sort_util(agent, reverse_edges, [], [])[:-1]

        inferior_queue.reverse()
        prior_queue.reverse()

        return inferior_queue, prior_queue

    def create_graph(self):
        """Create two graphs using the priority order """

        edges = defaultdict(list)
        reverse_edges = defaultdict(list)
        for i, j in self.priority_order:
            if i in edges.keys():
                edges[i].append(j)
            else:
                edges[i] = [j]

            if j in reverse_edges.keys():
                reverse_edges[j].append(i)
            else:
                reverse_edges[j] = [i]

        return edges, reverse_edges

    def prior_agent_collision_exist(self, inferior_agent, prior_agents):
        """Check if the current node has a collision between an agent and one of his prior agents"""

        for collision in self.collisions:
            if collision.agent1 == inferior_agent and collision.agent2 in prior_agents or \
               collision.agent2 == inferior_agent and collision.agent1 in prior_agents:
                return True
        return False


class PBSInput(CBSInput):
    def __init__(self, map_instance: MapfInstance, starts_list: List[WayPoint], goals_list: List[WayPoint],
                 initial_order=None):
        super().__init__(map_instance, starts_list, goals_list)
        if initial_order is None:
            initial_order = []
        if not isinstance(initial_order, list):
            raise WrongInput('Initial order isn\'t a list')
        self.initial_order = initial_order


class PBSOutput(MAPFOutput):
    def __init__(self, paths: List[Path], sum_of_costs=0, make_span=0, cpu_time=0.0, priority_order=None):
        super().__init__(paths, sum_of_costs, make_span, cpu_time)
        self.priority_order = priority_order


class PBSSolver(CBSSolver):
    """PBS high-level search."""
    def solve(self, pbs_input: PBSInput) -> PBSOutput:
        pbs_input.validate_input()
        self.start_time = timer.time()
        root = PBSNode([], [], pbs_input.initial_order, pbs_input.num_of_agents)
        for agent in range(pbs_input.num_of_agents):
            if not self.__update_plan(root, agent, pbs_input):
                raise NoSolution()
        root.update_node()
        self.push_node(root)
        while len(self.open_list) != 0:
            smallest_node = self.__pop_node()

            if timer.time() - self.start_time > self.time_limit * 1.2:
                raise TimeLimit(PBSOutput(smallest_node.paths, smallest_node.sum_of_costs,
                                          smallest_node.make_span, self.CPU_time, smallest_node.priority_order))

            smallest_node.detect_collisions()
            if len(smallest_node.collisions) == 0:
                self.CPU_time = timer.time() - self.start_time
                return PBSOutput(smallest_node.paths, smallest_node.sum_of_costs, smallest_node.make_span,
                                 self.CPU_time, smallest_node.priority_order)
            collision = smallest_node.collisions.pop()
            constraints = smallest_node.standard_splitting(collision)
            if len(constraints) == 0:
                continue
            node_one = self.__generate_child_node((collision.agent1, collision.agent2), constraints, smallest_node, pbs_input)
            node_two = self.__generate_child_node((collision.agent2, collision.agent1), constraints, smallest_node, pbs_input)
            if node_one:
                self.push_node(node_one)
            if node_two:
                self.push_node(node_two)
        raise NoSolution()

    def __generate_child_node(self, agents_priority, constraints, smallest_node, pbs_input):
        if agents_priority in smallest_node.priority_order or (agents_priority[1], agents_priority[0]) in smallest_node.priority_order:
            return None
        new_constraints = copy.deepcopy(smallest_node.constraints)
        # prior_agent = agents_priority[0]
        inferior_agent = agents_priority[1]
        for constraint in constraints:  # add only constraints for the inferior agent
            if constraint.agent == inferior_agent:
                new_constraints.append(constraint)
                list(set(new_constraints))
        new_ordering = copy.deepcopy(smallest_node.priority_order)
        new_ordering.append(agents_priority)
        new_ordering = list(set(new_ordering))
        new_node = PBSNode(new_constraints, copy.deepcopy(smallest_node.paths), new_ordering,
                           pbs_input.num_of_agents, smallest_node.sum_of_costs, smallest_node.make_span)
        if self.__update_plan(new_node, inferior_agent, pbs_input):
            new_node.update_node()
        return new_node

    """Build each agent a path that doesn't collide with his prior agents"""
    def __update_plan(self, node: PBSNode, i, pbs_input):
        inferior_agents, prior_agents = node.topological_sort(i)

        # if an initial order is given, plan paths for the prior agents that still doesn't exist in this stage
        if node.paths[i] is None:
            for idx, prior_agent in enumerate(prior_agents):
                for prior in prior_agents[idx + 1:]:
                    if not node.paths[prior]:
                        node.paths[prior] = self.__plan_path([], prior, pbs_input)
                constraints = node.create_prior_agent_constraints(prior_agents[idx + 1:], prior_agent)
                constraints += node.constraints
                path = self.__plan_path(constraints, prior_agent, pbs_input)
                if path is None:
                    return False
                node.paths[prior_agent] = path

        for idx, j in enumerate(inferior_agents):
            if idx > 0:
                prior_agents.append(inferior_agents[idx - 1])  # add agents to the prior list during the progress
            if j == i or node.prior_agent_collision_exist(j, prior_agents):
                # if an initial order is given, plan paths for the prior agent
                if not node.paths[j]:
                    path = self.__plan_path([], j, pbs_input)
                    if path is None:
                        return False
                    node.paths[j] = path

                constraints = node.create_prior_agent_constraints(prior_agents, j)
                constraints += node.constraints
                path = self.__plan_path(constraints, j, pbs_input)
                if path is None:
                    return False
                node.paths[j] = path
                node.detect_collisions()
                node.collisions = []
        return True

    """Build a single path for an agent"""
    def __plan_path(self, constraints, agent, pbs_input):
        goal = pbs_input.goals_list[agent]
        if goal not in self.heuristics.keys():
            self.build_heuristics_for_goal(goal, pbs_input.map_instance)
        lls_input = self.low_level_search.input_factory(pbs_input.map_instance, pbs_input.starts_list[agent],
                                                        goal, 0, agent, constraints,
                                                        self.heuristics[goal])
        return self.low_level_search.search(lls_input)

    def __pop_node(self) -> PBSNode:
        _, _, _, _, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node
