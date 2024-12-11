from single_agent_planner import move

class MDD: 
    def __init__(self, agents):
        self.num_agents = agents
        self.mdd = {
            agent: {} for agent in range(agents)
        }
        # self.timesteps = timesteps
        # self.start_location = start_location
        # self.goal_location = goal_location
        # self.my_map = my_map
        # self.mdd = dict()

    def add_path(self, agent, path, timestep):
        if timestep not in self.mdd[agent]:
            self.mdd[agent][timestep] = path
        else:
            self.mdd[agent][timestep].append(path)

        # Sort inner dictionary in the order of timesteps
        for agent in self.mdd:
            self.mdd[agent] = {key: self.mdd[agent][key] for key in sorted(self.mdd[agent])}

        print(self.mdd)
    # construct an MDD for an agent with all of the possible moves
    def generate_mdd(self):
        # Set root node in MDD to the start location of agent
        self.mdd[0] = self.start_location 
        for time in range(1, self.timesteps): 
            print(f"Time: {time}") #TEST
            possible_moves = [] 
            for location in self.mdd[time-1]: 
                # Find possible moves that allows an agent to progress 
                for i in range(4): 
                    next_location = move(location, i)
                    # make sure all moves are within bounds
                    if location[0] < 0 or location[0] >= len(self.my_map): 
                        continue
                    elif location[1] < 0 or location[1] >= len(self.my_map[0]): 
                        continue
                    else: 
                        if location not in possible_moves: 
                            possible_moves.append(next_location) 
                            print(f"Location: {location}, Possible Moves: {possible_moves}") #TEST
                if location == self.goal_location: 
                    print(f"Goal reached at timestep {time}")  #TEST
                    return self.mdd

            self.mdd[time] = possible_moves


