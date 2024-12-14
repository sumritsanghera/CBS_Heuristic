from single_agent_planner import move

class MDD: 
    def __init__(self, my_map, start_location, goal_location, timesteps): 
        self.start_location = start_location
        self.timesteps = timesteps
        self.goal_location = goal_location
        self.my_map = my_map
        self.mdd = dict()

    def check_distance(self, start, loc, goal):
        old_distance = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        new_distance = abs(loc[0] - goal[0]) + abs(loc[1] - goal[1])
        if new_distance <= old_distance: # new location is closer to goal
            return True
        return False

    # construct an MDD for an agent with all of the possible moves
    def generate_mdd(self):
        # Set root node in MDD to the start location of agent
        self.mdd[0] = [self.start_location]
        time = 1
        while(time <= self.timesteps):
            #print(f"Time: {time}") #:TEST
            possible_moves = [] 
            for location in self.mdd[time-1]: 
                # Find possible moves that allows an agent to progress 
                for i in range(4): 
                    next_location = move(location, i)
                    # make sure all moves are within bounds
                    if (next_location[0] < 0 or next_location[0] >= len(self.my_map) 
                        or next_location[1] < 0 or next_location[1] >= len(self.my_map)):
                        continue
                    ## avoid adding obstabcles
                    if self.my_map[next_location[0]][next_location[1]]: 
                        continue
                    ## check if move makes agent closer to goal or not before adding to MDD
                    if (self.check_distance(location, next_location, self.goal_location) 
                        and next_location not in possible_moves):
                        print(next_location)
                        possible_moves.append(next_location) 
                        #print(f"Location: {location}, Possible Moves: {possible_moves}") #TEST

                if location == self.goal_location: 
                    #print(f"Goal reached at timestep {time}")  #TEST
                    return self.mdd

            self.mdd[time] = possible_moves
            time+=1

    def is_dependent(self, other): 
        agent1_len = len(self.mdd)
        agent2_len = len(other.mdd)
        for i in range(max(agent1_len, agent2_len)): 
            for loc1 in self.mdd[i]: 
                for loc2 in other.mdd[i]: 
                    # cardinal conflict (direct collision)
                    if (loc1 == loc2): 
                        return True

        return False




