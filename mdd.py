from single_agent_planner import move

class MDD: 
    def __init__(self, my_map, start_location, goal_location):
        self.start_location = start_location
        self.goal_location = goal_location
        self.my_map = my_map
        self.mdd = self.generate_mdd()

    def check_distance(self, start, loc, goal):
        old_distance = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        new_distance = abs(loc[0] - goal[0]) + abs(loc[1] - goal[1])
        if new_distance <= old_distance: # new location is closer to goal
            return True
        return False

    # construct an MDD for an agent with all of the possible moves
    def generate_mdd(self) -> dict:
        # Set root node in MDD to the start location of agent
        self.mdd = dict()
        self.mdd[0] = [self.start_location]
        time = 1
        while(True):
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
                    ## avoid adding obstacles
                    if self.my_map[next_location[0]][next_location[1]]: 
                        continue
                    ## check if move makes agent closer to goal or not before adding to MDD
                    if (self.check_distance(location, next_location, self.goal_location) 
                        and next_location not in possible_moves):
                        possible_moves.append(next_location) 
                        #print(f"Location: {location}, Possible Moves: {possible_moves}") #TEST

                if location == self.goal_location: 
                    #print(f"Goal reached at timestep {time}")  #TEST
                    print(f"check mdd: {self.mdd}")
                    return self.mdd

            self.mdd[time] = possible_moves
            time+=1

    def is_dependent(self, other):
        t_max = max(len(self.mdd), len(other.mdd))

        joint_mdd = {
            timestep: [] for timestep in range(t_max)
        }

        joint_mdd[0].append([self.mdd[0][0], other.mdd[0][0]])

        for t in range(t_max):
            for pair in joint_mdd[t]:
                children_1 = self.mdd[t+1]
                children_2 = other.mdd[t+1]
                for child_1 in children_1:
                    # Check if in parent-child relationship
                    if abs(child_1[0]-pair[0][0]) + abs(child_1[1]-pair[0][1]) == 1:
                        for child_2 in children_2:
                            if abs(child_2[0] - pair[1][0]) + abs(child_2[1] - pair[1][1]) == 1:
                                if child_1 != child_2 and [child_1, child_2] not in joint_mdd[t+1]:
                                    joint_mdd[t+1].append([child_1, child_2])
            # Check Dependency
            if len(joint_mdd[t]) == 0:
                return True
        return False




