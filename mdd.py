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
            self.mdd[agent][timestep] = [path]
        else:
            if path not in self.mdd[agent][timestep]:
                self.mdd[agent][timestep].append(path)
        print(f"\n{self.mdd}\n")

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
    
    def get_locations(self, agent, timestep):
        if timestep in self.mdd[agent]:
            return self.mdd[agent][timestep]
        return []
    
    def merge_mdds(self, a1, a2, timestep):
        if timestep not in self.mdd[a1] or timestep not in self.mdd[a2]:
            return None
        
        #get all locations for both agents at this timestep
        locations1 = self.mdd[a1][timestep]
        locations2 = self.mdd[a2][timestep]
        
        #try to find a pair of locations that don't conflict
        for loc1 in locations1:
            for loc2 in locations2:
                #vertex conflixts
                if loc1 != loc2:
                    #check for edge conflicts with previous timestep
                    if timestep > 0:
                        prev_locs1 = self.mdd[a1][timestep-1]
                        prev_locs2 = self.mdd[a2][timestep-1]
                        edge_conflict = False
                        for prev1 in prev_locs1:
                            for prev2 in prev_locs2:
                                if prev1 == loc2 and prev2 == loc1:
                                    edge_conflict = True
                                    break
                            if edge_conflict:
                                break
                        if not edge_conflict:
                            return True  #found non-conflicting positions
                    else:
                        return True 
        
        return False #no nonconflict pos found

    def is_cardinal_conflict(self, collision): #cardinal = iff both agents have no other path at same timestep to avoid conflict
        timestep = collision['timestep']
        a1 = collision['a1']
        a2 = collision['a2']
        
        #get all possible locations for both agents at this timestep
        locs1 = self.get_locations(a1, timestep)
        locs2 = self.get_locations(a2, timestep)
        
        # #if we don't have MDD data for this timestep, assume non-cardinal
        # if not locs1 or not locs2:
        #     return False
            
        # For vertex conflicts
        if len(collision['loc']) == 1:
            conflict_loc = collision['loc'][0]
            
            # Check if either agent has alternative locations at this timestep
            alt_locs1 = [loc for loc in locs1 if loc != conflict_loc]
            alt_locs2 = [loc for loc in locs2 if loc != conflict_loc]
            
            #if either agent has other alternatives -> not cardinal
            if alt_locs1 or alt_locs2:
                return False
                
            #if no agent has alternatives -> cardinal
            return True
                
        # For edge conflicts
        else:
            loc1_from, loc1_to = collision['loc']
            loc2_from, loc2_to = loc2_from, loc2_to = collision['loc'][::-1] # reverse for agent 2
            
            #get previous timestep locations
            prev_locs1 = self.get_locations(a1, timestep-1) if timestep > 0 else []
            prev_locs2 = self.get_locations(a2, timestep-1) if timestep > 0 else []
            
            #check if either agent has alternative paths
            has_alt1 = any(prev != loc1_from or curr != loc1_to 
                        for prev in prev_locs1 for curr in locs1)
            has_alt2 = any(prev != loc2_from or curr != loc2_to 
                        for prev in prev_locs2 for curr in locs2)
            
            #if either agent has alternatives, it's not cardinal
            return not (has_alt1 or has_alt2)

