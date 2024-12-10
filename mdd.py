from single_agent_planner import move

class MDD: 
    def __init__(self, timesteps, my_map, start_location, goal_location): 
        self.timesteps = timesteps
        self.start_location = start_location
        self.goal_location = goal_location
        self.my_map = my_map
        self.mdd = dict()

    # construct an MDD for an agent with all of the possible moves
    def genereate_mdd(self):
        # Set root node in MDD to the start location of agent
        self.mdd[0] = self.start_location 
        for time in range(1, self.timesteps): 
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
                if location == self.goal_location: 
                    return self.mdd

            self.mdd[time] = possible_moves


