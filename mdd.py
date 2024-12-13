# from single_agent_planner import move

# class MDD: 
#     def __init__(self, agents):
#         self.num_agents = agents
#         self.mdd = {
#             agent: {} for agent in range(agents)
#         }
#         # self.timesteps = timesteps
#         # self.start_location = start_location
#         # self.goal_location = goal_location
#         # self.my_map = my_map
#         # self.mdd = dict()

#     def add_path(self, agent, path, timestep):
#         if timestep not in self.mdd[agent]:
#             self.mdd[agent][timestep] = [path]
#         else:
#             if path not in self.mdd[agent][timestep]:
#                 self.mdd[agent][timestep].append(path)
#         print(f"\n{self.mdd}\n")

#     # construct an MDD for an agent with all of the possible moves
#     def generate_mdd(self):
#         # Set root node in MDD to the start location of agent
#         self.mdd[0] = self.start_location 
#         for time in range(1, self.timesteps): 
#             print(f"Time: {time}") #TEST
#             possible_moves = [] 
#             for location in self.mdd[time-1]: 
#                 # Find possible moves that allows an agent to progress 
#                 for i in range(4): 
#                     next_location = move(location, i)
#                     # make sure all moves are within bounds
#                     if location[0] < 0 or location[0] >= len(self.my_map): 
#                         continue
#                     elif location[1] < 0 or location[1] >= len(self.my_map[0]): 
#                         continue
#                     else: 
#                         if location not in possible_moves: 
#                             possible_moves.append(next_location) 
#                             print(f"Location: {location}, Possible Moves: {possible_moves}") #TEST
#                 if location == self.goal_location: 
#                     print(f"Goal reached at timestep {time}")  #TEST
#                     return self.mdd

#             self.mdd[time] = possible_moves
    
#     def get_locations(self, agent, timestep):
#         if timestep in self.mdd[agent]:
#             return self.mdd[agent][timestep]
#         return []
    
#     def merge_mdds(self, a1, a2, timestep):
#         if timestep not in self.mdd[a1] or timestep not in self.mdd[a2]:
#             return None
        
#         #get all locations for both agents at this timestep
#         locations1 = self.mdd[a1][timestep]
#         locations2 = self.mdd[a2][timestep]
        
#         #try to find a pair of locations that don't conflict
#         for loc1 in locations1:
#             for loc2 in locations2:
#                 #vertex conflixts
#                 if loc1 != loc2:
#                     #check for edge conflicts with previous timestep
#                     if timestep > 0:
#                         prev_locs1 = self.mdd[a1][timestep-1]
#                         prev_locs2 = self.mdd[a2][timestep-1]
#                         edge_conflict = False
#                         for prev1 in prev_locs1:
#                             for prev2 in prev_locs2:
#                                 if prev1 == loc2 and prev2 == loc1:
#                                     edge_conflict = True
#                                     break
#                             if edge_conflict:
#                                 break
#                         if not edge_conflict:
#                             return True  #found non-conflicting positions
#                     else:
#                         return True 
        
#         return False #no nonconflict pos found

#     def is_cardinal_conflict(self, collision): #cardinal = iff both agents have no other path at same timestep to avoid conflict
#         timestep = collision['timestep']
#         a1 = collision['a1']
#         a2 = collision['a2']
        
#         #get all possible locations for both agents at this timestep
#         locs1 = self.get_locations(a1, timestep)
#         locs2 = self.get_locations(a2, timestep)
        
#         # #if we don't have MDD data for this timestep, assume non-cardinal
#         # if not locs1 or not locs2:
#         #     return False
            
#         # For vertex conflicts
#         if len(collision['loc']) == 1:
#             conflict_loc = collision['loc'][0]
            
#             # Check if either agent has alternative locations at this timestep
#             alt_locs1 = [loc for loc in locs1 if loc != conflict_loc]
#             alt_locs2 = [loc for loc in locs2 if loc != conflict_loc]
            
#             #if either agent has other alternatives -> not cardinal
#             if alt_locs1 or alt_locs2:
#                 return False
                
#             #if no agent has alternatives -> cardinal
#             return True
                
#         # For edge conflicts
#         else:
#             loc1_from, loc1_to = collision['loc']
#             loc2_from, loc2_to = loc2_from, loc2_to = collision['loc'][::-1] # reverse for agent 2
            
#             #get previous timestep locations
#             prev_locs1 = self.get_locations(a1, timestep-1) if timestep > 0 else []
#             prev_locs2 = self.get_locations(a2, timestep-1) if timestep > 0 else []
            
#             #check if either agent has alternative paths
#             has_alt1 = any(prev != loc1_from or curr != loc1_to 
#                         for prev in prev_locs1 for curr in locs1)
#             has_alt2 = any(prev != loc2_from or curr != loc2_to 
#                         for prev in prev_locs2 for curr in locs2)
            
#             #if either agent has alternatives, it's not cardinal
#             return not (has_alt1 or has_alt2)
from single_agent_planner import *
# class MDD:
#     def __init__(self, my_map, start_loc, goal_loc, h_values, agent, constraints, cost):
#         self.my_map = my_map
#         self.start_loc = start_loc
#         self.goal_loc = goal_loc
#         self.h_values = h_values
#         self.agent = agent
#         self.constraints = constraints
#         self.cost = cost
#         self.mdd = {i: [] for i in range(cost + 1)}
#         self.mdd_edges = {i: {} for i in range(cost)}
#         self.generate_mdd()

#     def generate_mdd(self):
#         # Initialize root level
#         self.mdd[0] = [self.start_loc]
#         constraint_table = build_constraint_table(self.constraints, self.agent)
        
#         # Build MDD level by level
#         for level in range(self.cost):
#             for loc in self.mdd[level]:
#                 self.mdd_edges[level][loc] = []
#                 # Try all possible moves including wait
#                 for dir in range(5):
#                     if dir == 4:  # wait action
#                         next_loc = loc
#                     else:
#                         next_loc = move(loc, dir)
                    
#                     # Check if move is valid
#                     if next_loc[0] < 0 or next_loc[0] >= len(self.my_map) \
#                        or next_loc[1] < 0 or next_loc[1] >= len(self.my_map[0]):
#                         continue
#                     if self.my_map[next_loc[0]][next_loc[1]]:
#                         continue
#                     if is_constrained(loc, next_loc, level + 1, constraint_table):
#                         continue
                    
#                     # Check if next location can reach goal within remaining cost
#                     if level + 1 + self.h_values[next_loc] <= self.cost:
#                         if next_loc not in self.mdd[level + 1]:
#                             self.mdd[level + 1].append(next_loc)
#                         self.mdd_edges[level][loc].append(next_loc)

#         # Remove dead-end nodes
#         for level in range(self.cost - 1, -1, -1):
#             valid_locs = []
#             for loc in self.mdd[level]:
#                 if len(self.mdd_edges[level][loc]) > 0:
#                     valid_locs.append(loc)
#             self.mdd[level] = valid_locs

# def detect_cardinal_conflicts(collision, mdds):
#     """Classify conflict as cardinal, semi-cardinal, or non-cardinal"""
#     timestep = collision['timestep']
#     a1 = collision['a1']
#     a2 = collision['a2']
    
#     # Get MDD levels for both agents at collision timestep
#     mdd1_level = mdds[a1].mdd[timestep] if timestep < len(mdds[a1].mdd) else []
#     mdd2_level = mdds[a2].mdd[timestep] if timestep < len(mdds[a2].mdd) else []
    
#     if len(collision['loc']) == 1:  # vertex collision
#         loc = collision['loc'][0]
#         # Cardinal if both agents must use the location (single option in MDD)
#         if len(mdd1_level) == 1 and len(mdd2_level) == 1 and \
#            mdd1_level[0] == loc and mdd2_level[0] == loc:
#             return "cardinal"
#         # Semi-cardinal if one agent must use the location
#         elif (len(mdd1_level) == 1 and mdd1_level[0] == loc) or \
#              (len(mdd2_level) == 1 and mdd2_level[0] == loc):
#             return "semi-cardinal"
#         # Non-cardinal otherwise
#         return "non-cardinal"
#     else:  # edge collision
#         loc1, loc2 = collision['loc']
#         # Check if edge transition exists in both MDDs
#         edge1_exists = any(loc2 in mdds[a1].mdd_edges[timestep-1].get(loc1, []))
#         edge2_exists = any(loc1 in mdds[a2].mdd_edges[timestep-1].get(loc2, []))
        
#         if edge1_exists and edge2_exists:
#             if len(mdd1_level) == 1 and len(mdd2_level) == 1:
#                 return "cardinal"
#             elif len(mdd1_level) == 1 or len(mdd2_level) == 1:
#                 return "semi-cardinal"
#         return "non-cardinal"

# def build_joint_mdd(mdd1, mdd2):
#     """Build joint MDD for two agents"""
#     joint_mdd = {}
#     max_depth = min(len(mdd1.mdd), len(mdd2.mdd))
    
#     # Initialize joint MDD
#     for level in range(max_depth):
#         joint_mdd[level] = []
        
#     # Add root node
#     joint_mdd[0].append((mdd1.mdd[0][0], mdd2.mdd[0][0]))
    
#     # Build joint MDD level by level
#     for level in range(max_depth - 1):
#         for loc1, loc2 in joint_mdd[level]:
#             # Get possible next locations for both agents
#             next_locs1 = mdd1.mdd_edges[level].get(loc1, [])
#             next_locs2 = mdd2.mdd_edges[level].get(loc2, [])
            
#             # Add valid combinations to next level
#             for next1 in next_locs1:
#                 for next2 in next_locs2:
#                     # Check for conflicts
#                     if next1 != next2:  # No vertex conflict
#                         if not (next1 == loc2 and next2 == loc1):  # No edge conflict
#                             joint_mdd[level + 1].append((next1, next2))
    
#     return joint_mdd

import copy
import random

import collections
class MDD:
    def __init__(self, my_map, start_loc, goal_loc, h_values, agent, constraints, cost):
        self.my_map = my_map
        self.start_loc = start_loc
        self.goal_loc = goal_loc
        self.h_values = h_values
        self.agent = agent
        self.constraints = constraints
        self.cost = cost
        self.mdd = {i: [] for i in range(cost + 1)}
        self.mdd_edges = {i: {} for i in range(cost)}
        self.generate_mdd()

    def generate_mdd(self):
        """Generate MDD following the same structure as build_mdd in the reference code"""
        constraint_table = build_constraint_table(self.constraints, self.agent)
        
        # Initialize root node
        root_node = {'prev': None, 'cost': 0, 'curr_loc': self.start_loc}
        open_list = collections.deque([root_node])
        
        # Use dictionary to track nodes at each level
        for i in range(self.cost + 1):
            self.mdd[i] = []
            if i < self.cost:
                self.mdd_edges[i] = {}

        while open_list:
            current_node = open_list.popleft()
            
            # If we reach goal with exact cost, add the path to MDD
            if current_node['curr_loc'] == self.goal_loc and current_node['cost'] == self.cost:
                curr = current_node
                index = self.cost
                
                # Add nodes to MDD by backtracking
                while curr is not None:
                    if curr['curr_loc'] not in self.mdd[index]:
                        self.mdd[index].append(curr['curr_loc'])
                    if curr['prev'] is not None:
                        level = index - 1
                        prev_loc = curr['prev']['curr_loc']
                        if prev_loc not in self.mdd_edges[level]:
                            self.mdd_edges[level][prev_loc] = []
                        if curr['curr_loc'] not in self.mdd_edges[level][prev_loc]:
                            self.mdd_edges[level][prev_loc].append(curr['curr_loc'])
                    curr = curr['prev']
                    index -= 1
                continue

            # Try all possible moves including wait
            for dir in range(5):
                if dir == 4:  # wait action
                    next_loc = current_node['curr_loc']
                else:
                    next_loc = move(current_node['curr_loc'], dir)
                
                # Check if move is valid
                if next_loc[0] < 0 or next_loc[0] >= len(self.my_map) \
                   or next_loc[1] < 0 or next_loc[1] >= len(self.my_map[0]):
                    continue
                if self.my_map[next_loc[0]][next_loc[1]]:
                    continue
                if is_constrained(current_node['curr_loc'], next_loc, 
                                current_node['cost'] + 1, constraint_table):
                    continue
                
                child_node = {
                    'prev': current_node,
                    'cost': current_node['cost'] + 1,
                    'curr_loc': next_loc
                }
                
                # Check if path can reach goal within cost bound
                if child_node['cost'] + self.h_values[next_loc] <= self.cost:
                    open_list.append(child_node)

def detect_cardinal_conflicts(collision, mdds):
    """Classify conflict following the same logic as return_optimal_conflict"""
    timestep = collision['timestep']
    a1 = collision['a1']
    a2 = collision['a2']
    
    mdd1 = mdds[a1].mdd
    mdd2 = mdds[a2].mdd
    
    # Check if timestep exceeds MDD depth
    if timestep >= len(mdd1) or timestep >= len(mdd2):
        return "non-cardinal"
    
    if len(collision['loc']) == 1:  # vertex collision
        loc = collision['loc'][0]
        
        # Cardinal conflict
        if (len(mdd1[timestep]) == 1 and len(mdd2[timestep]) == 1 and 
            mdd1[timestep][0] == loc and mdd2[timestep][0] == loc):
            return "cardinal"
            
        # Semi-cardinal conflict
        if ((len(mdd1[timestep]) == 1 or len(mdd2[timestep]) == 1) and 
            loc in mdd1[timestep] and loc in mdd2[timestep]):
            return "semi-cardinal"
            
        # Non-cardinal conflict
        return "non-cardinal"
        
    else:  # edge collision
        loc1, loc2 = collision['loc']
        prev_timestep = timestep - 1
        
        # Check if edge exists in both MDDs
        edge1_exists = False
        if prev_timestep in mdds[a1].mdd_edges:
            if loc1 in mdds[a1].mdd_edges[prev_timestep]:
                edge1_exists = loc2 in mdds[a1].mdd_edges[prev_timestep][loc1]
                
        edge2_exists = False
        if prev_timestep in mdds[a2].mdd_edges:
            if loc2 in mdds[a2].mdd_edges[prev_timestep]:
                edge2_exists = loc1 in mdds[a2].mdd_edges[prev_timestep][loc2]
        
        if edge1_exists and edge2_exists:
            if len(mdd1[timestep]) == 1 and len(mdd2[timestep]) == 1:
                return "cardinal"
            elif len(mdd1[timestep]) == 1 or len(mdd2[timestep]) == 1:
                return "semi-cardinal"
                
        return "non-cardinal"

def build_joint_mdd(mdd1, mdd2):
    """Build joint MDD following the same logic as join_MDD"""
    max_depth = min(len(mdd1.mdd), len(mdd2.mdd))
    joint_mdd = {i: [] for i in range(max_depth)}
    
    # Add root node
    joint_mdd[0] = [(mdd1.mdd[0][0], mdd2.mdd[0][0])]
    
    # Build level by level
    for level in range(max_depth - 1):
        for loc1, loc2 in joint_mdd[level]:
            next_locs1 = mdd1.mdd_edges[level].get(loc1, [])
            next_locs2 = mdd2.mdd_edges[level].get(loc2, [])
            
            for next1 in next_locs1:
                for next2 in next_locs2:
                    if next1 != next2:  # No vertex conflict
                        if not (next1 == loc2 and next2 == loc1):  # No edge conflict
                            joint_mdd[level + 1].append((next1, next2))
    
    return joint_mdd