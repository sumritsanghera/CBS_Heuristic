import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = [
            # {
            #     'agent': 0,
            #     'loc': [(1, 5)],
            #     'timestep': 4
            # }  #1.2

            # {
            #     'agent': 1,
            #     'loc': [(1,2), (1,3)],
            #     'timestep': 1
            # }

            # {'agent': 1, 'loc': [(1, 2), (1, 3)], 'timestep': 1} #1.3

            # {
            #     'agent': 0,
            #     'loc': [(1,5)],
            #     'timestep': 10
            # }  #1.4

            # {
            #     'agent': 1,
            #     'loc': [(1,3), (1,4)],
            #     'timestep': 2
            # },
            # {
            #     'agent': 1,
            #     'loc': [(1,2)],
            #     'timestep': 2
            # },
            # {
            #     'agent': 1,
            #     'loc': [(1,3)],
            #     'timestep': 2
            # } ## 1.5 blue goes down
        
        ]

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            
            #2.1
            # for t, location in enumerate(path):
            # # Loop over all future agents to apply vertex constraints
            #     for future_agent in range(i + 1, self.num_of_agents):
            #         constraint = {
            #             'agent': future_agent,
            #             'loc': [location],  # Vertex constraint location for future agents
            #             'timestep': t       # Time step when the location is constrained
            #         }
            #         constraints.append(constraint)

            for agent in range(self.num_of_agents):
                if agent == i:
                    continue  # Skip constraints for the current agent

                for index, location in enumerate(path):
                    # Task 2.1: Add vertex constraints
                    constraints.append({
                        'agent': agent,
                        'loc': [location],
                        'timestep': index
                    })

                    # Task 2.2: Add edge constraints (forward and backward)
                    if index > 0:
                        prev_location = path[index - 1]
                        constraints.append({
                            'agent': agent,
                            'loc': [prev_location, location],
                            'timestep': index
                        })
                        constraints.append({
                            'agent': agent,
                            'loc': [location, prev_location],
                            'timestep': index
                        })

                    # Task 2.3: Add additional goal constraints for future timesteps
                    goal_location = path[-1]
                    for goal_constraint in range(len(path), self.num_of_agents * len(path)):
                        constraints.append({
                            'agent': agent,
                            'loc': [goal_location],
                            'timestep': goal_constraint
                        })

            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
