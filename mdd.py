#from single_agent_planner import *
#import collections
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
#         constraint_table = build_constraint_table(self.constraints, self.agent)
        
#         root_node = {'prev': None, 'cost': 0, 'curr_loc': self.start_loc}
#         open_list = collections.deque([root_node])
        
#         for i in range(self.cost + 1):
#             self.mdd[i] = []
#             if i < self.cost:
#                 self.mdd_edges[i] = {}

#         while open_list:
#             current_node = open_list.popleft()
            
#             #if reached goal with exact cost, add the path to MDD
#             if current_node['curr_loc'] == self.goal_loc and current_node['cost'] == self.cost:
#                 curr = current_node
#                 index = self.cost
                
#                 #add nodes to MDD by backtracking
#                 while curr is not None:
#                     if curr['curr_loc'] not in self.mdd[index]:
#                         self.mdd[index].append(curr['curr_loc'])
#                     if curr['prev'] is not None:
#                         level = index - 1
#                         prev_loc = curr['prev']['curr_loc']
#                         if prev_loc not in self.mdd_edges[level]:
#                             self.mdd_edges[level][prev_loc] = []
#                         if curr['curr_loc'] not in self.mdd_edges[level][prev_loc]:
#                             self.mdd_edges[level][prev_loc].append(curr['curr_loc'])
#                     curr = curr['prev']
#                     index -= 1
#                 continue

#             #try all possible moves including wait
#             for dir in range(5):
#                 if dir == 4:  # wait action
#                     next_loc = current_node['curr_loc']
#                 else:
#                     next_loc = move(current_node['curr_loc'], dir)
                
#                 #check if valid
#                 if next_loc[0] < 0 or next_loc[0] >= len(self.my_map) \
#                    or next_loc[1] < 0 or next_loc[1] >= len(self.my_map[0]):
#                     continue
#                 if self.my_map[next_loc[0]][next_loc[1]]:
#                     continue
#                 if is_constrained(current_node['curr_loc'], next_loc, 
#                                 current_node['cost'] + 1, constraint_table):
#                     continue
                
#                 child_node = {
#                     'prev': current_node,
#                     'cost': current_node['cost'] + 1,
#                     'curr_loc': next_loc
#                 }
                
#                 if child_node['cost'] + self.h_values[next_loc] <= self.cost:
#                     open_list.append(child_node)

# def detect_cardinal_conflicts(collision, mdds):
#     timestep = collision['timestep']
#     a1 = collision['a1']
#     a2 = collision['a2']
    
#     mdd1 = mdds[a1].mdd
#     mdd2 = mdds[a2].mdd
    
#     #check if timestep exceeds MDD depth
#     if timestep >= len(mdd1) or timestep >= len(mdd2):
#         return "non-cardinal"
    
#     if len(collision['loc']) == 1:  # vertex collision
#         loc = collision['loc'][0]
        
#         # Cardinal conflict
#         if (len(mdd1[timestep]) == 1 and len(mdd2[timestep]) == 1 and 
#             mdd1[timestep][0] == loc and mdd2[timestep][0] == loc):
#             return "cardinal"
            
#         # Semi-cardinal conflict
#         if ((len(mdd1[timestep]) == 1 or len(mdd2[timestep]) == 1) and 
#             loc in mdd1[timestep] and loc in mdd2[timestep]):
#             return "semi-cardinal"
            
#         # Non-cardinal conflict
#         return "non-cardinal"
        
#     else:  # edge collision
#         loc1, loc2 = collision['loc']
#         prev_timestep = timestep - 1
        
#         # Check if edge exists in both MDDs
#         edge1_exists = False
#         if prev_timestep in mdds[a1].mdd_edges:
#             if loc1 in mdds[a1].mdd_edges[prev_timestep]:
#                 edge1_exists = loc2 in mdds[a1].mdd_edges[prev_timestep][loc1]
                
#         edge2_exists = False
#         if prev_timestep in mdds[a2].mdd_edges:
#             if loc2 in mdds[a2].mdd_edges[prev_timestep]:
#                 edge2_exists = loc1 in mdds[a2].mdd_edges[prev_timestep][loc2]
        
#         if edge1_exists and edge2_exists:
#             if len(mdd1[timestep]) == 1 and len(mdd2[timestep]) == 1:
#                 return "cardinal"
#             elif len(mdd1[timestep]) == 1 or len(mdd2[timestep]) == 1:
#                 return "semi-cardinal"
                
#         return "non-cardinal"

# def build_joint_mdd(mdd1, mdd2):
#     max_depth = min(len(mdd1.mdd), len(mdd2.mdd))
#     joint_mdd = {i: [] for i in range(max_depth)}
    
#     #root node
#     joint_mdd[0] = [(mdd1.mdd[0][0], mdd2.mdd[0][0])]
    
#     #build ds level by level
#     for level in range(max_depth - 1):
#         for loc1, loc2 in joint_mdd[level]:
#             next_locs1 = mdd1.mdd_edges[level].get(loc1, [])
#             next_locs2 = mdd2.mdd_edges[level].get(loc2, [])
            
#             for next1 in next_locs1:
#                 for next2 in next_locs2:
#                     if next1 != next2:  # No vertex conflict
#                         if not (next1 == loc2 and next2 == loc1):  # No edge conflict
#                             joint_mdd[level + 1].append((next1, next2))
    
#     return joint_mdd

####NEW####

import math
from single_agent_planner import move, push_node, pop_node, compare_nodes, get_path
import heapq

# a_star search to find the distance from current location to goal
def get_distance(loc, goal, my_map, heuristics): 
    open_list = [] 
    closed_list = dict()
    root = {'loc': loc, 'g_val': 0, 'h_val': heuristics[loc], 'parent': None}
    push_node(open_list, root)
    closed_list[root['loc']] = root
    while len(open_list) > 0: 
        cur = pop_node(open_list)
        if cur['loc'] == goal: 
            return len(get_path(cur))
        for d in range(4): 
            new_loc = move(cur['loc'], d)
            # check for obstacles and grid constraints
            if (new_loc[0] < 0 or new_loc[0] >= len(my_map)
            or new_loc[1] < 0 or new_loc[1] >= len(my_map[0])):
                continue
            if my_map[new_loc[0]][new_loc[1]]:
                continue
            # add new child node
            child = {'loc': new_loc, 
                     'g_val': cur['g_val'] + 1, 
                     'h_val': heuristics[new_loc], 
                     'parent': cur, 
                     }
            if new_loc in closed_list: 
                existing = closed_list[new_loc]
                if (compare_nodes(child, existing)):
                    closed_list[new_loc] = child
                    push_node(open_list, child)
            else: 
                closed_list[new_loc] = child
                push_node(open_list, child)


# find best move based from lowest distance from location to goal
def optimal_paths(locs, goal, already_visited, my_map, heuristics) -> list:
    paths = dict()
    min_dist = math.inf
    for loc in locs:
        if loc in already_visited:
            continue
        dist = get_distance(loc, goal, my_map, heuristics)
        if dist not in paths:
            paths[dist] = [loc]
        elif loc not in paths[dist]:
            paths[dist].append(loc)
        if dist < min_dist:
            min_dist = dist
    if min_dist != math.inf:
        return paths[min_dist]
    else: 
        return []


class MDD:
    def __init__(self, my_map, loc_start, loc_goal, heuristics):
        self.loc_start = loc_start
        self.loc_goal = loc_goal
        self.my_map = my_map
        self.mdd = dict()
        self.heuristics = heuristics
        self.generate_mdd()

    # construct an MDD for an agent with all the possible moves
    def generate_mdd(self):
    # Set root node in MDD to the start location of agent
        self.mdd[0] = [self.loc_start]
        time = 1
        already_visited = []
        while True:
            # print(f"Time: {time}") #:TEST
            possible_moves = []
            for loc in self.mdd[time - 1]:
                if loc == self.loc_goal:
                    # print(f"Goal reached at timestep {time}")  #TEST
                    print(f"check mdd: {self.mdd}")
                    return
                # Find possible moves that allow an agent to progress
                already_visited.append(loc)
                for i in range(4):
                    loc_next = move(loc, i)
                    # make sure all moves are within bounds
                    if (loc_next[0] < 0 or loc_next[0] >= len(self.my_map)
                    or loc_next[1] < 0 or loc_next[1] >= len(self.my_map[0])):
                        continue
                    # avoid adding obstacles
                    if self.my_map[loc_next[0]][loc_next[1]]:
                        continue
                    if loc_next not in possible_moves:
                        possible_moves.append(loc_next)
                    # print(f"Location: {location}, Possible Moves: {possible_moves}") #TEST
            
                self.mdd[time] = optimal_paths(possible_moves, self.loc_goal, already_visited, self.my_map,
                                           self.heuristics)

            # self.mdd[time] = possible_moves
            time += 1

    # Returns dependency and cardinality
    def is_dependent(self, other) -> bool:
        t_min = min(len(self.mdd), len(other.mdd))

        joint_mdd = {
            timestep: [] for timestep in range(t_min)
        }

        joint_mdd[0].append([self.mdd[0][0], other.mdd[0][0]])

        for t in range(t_min -1 ):
            for pair in joint_mdd[t]:
                children_1 = self.mdd[t + 1]
                children_2 = other.mdd[t + 1]
                for child_1 in children_1:
                    # Check if in parent-child relationship
                    if abs(child_1[0] - pair[0][0]) + abs(child_1[1] - pair[0][1]) == 1:
                        for child_2 in children_2:
                            if abs(child_2[0] - pair[1][0]) + abs(child_2[1] - pair[1][1]) == 1:
                                if child_1 != child_2 and [child_1, child_2] not in joint_mdd[t + 1]:
                                    joint_mdd[t + 1].append([child_1, child_2])
            # Check Dependency and cardinality
            if len(joint_mdd[t]) == 0:
                if len(joint_mdd[t - 1]) == 1:
                    return True
                else:
                    return True
        return False
