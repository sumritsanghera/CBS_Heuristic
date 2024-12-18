import math
from single_agent_planner import *
import heapq
import time as timer

# a_star search to find the distance from current location to goal to generate_mdd
def get_distance(loc, goal, my_map, heuristics): 
    open_list = [] 
    closed_list = dict()
    root = {'loc': loc, 'g_val': 0, 'h_val': heuristics[loc], 'parent': None}
    push_node(open_list, root)
    closed_list[root['loc']] = root
    while len(open_list) > 0: 
        cur = pop_node(open_list)
        if cur and cur['loc'] == goal: 
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
        if dist is None: 
            continue
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
        self.already_visited = []
        self.generate_mdd()

    # construct an MDD for an agent with all the possible moves
    def generate_mdd(self):
    # Set root node in MDD to the start location of agent
        self.mdd[0] = [self.loc_start]
        time = 1
        while True:
            # print(f"Time: {time}") #:TEST
            possible_moves = []
            if self.mdd[time -1] == []: 
                return 
            for loc in self.mdd[time - 1]:
                if loc == self.loc_goal:
                    # print(f"Goal reached at timestep {time}")  #TEST
                    print(f"check mdd: {self.mdd}")
                    return
                # Find possible moves that allow an agent to progress
                self.already_visited.append(loc)
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
            
            self.mdd[time] = optimal_paths(possible_moves, self.loc_goal, self.already_visited, self.my_map,
                                           self.heuristics)

            # self.mdd[time] = possible_moves
            time += 1

    # Update MDD without having to reconstruct the whole MDD
    def update_mdd(self, agent, constraints, path): 
        if len(path) > len(self.mdd): 
            for i in range(len(path) - len(self.mdd)): 
                self.mdd[len(self.mdd)+i] = []
        # initialize extra space in MDD
        for constraint in constraints: 
            if constraint['agent'] == agent: 
                time = constraint['timestep']
                # check edge constraint 
                if len(constraint['loc']) == 2: 
                    if constraint['loc'][0] in self.mdd[time-1] and constraint['loc'][1] in self.mdd[time]: 
                        self.mdd[time-1].remove(constraint['loc'][0])
                        self.mdd[time].remove(constraint['loc'][1])
                else: 
                    if constraint['loc'][0] in self.mdd[time]: 
                        self.mdd[time].remove(constraint['loc'][0])
        for i, move in enumerate(path): 
            if move not in self.mdd[i]: 
                self.mdd[i].append(move)

        return self.mdd

    # Returns dependency and cardinality
    def is_dependent(self, other) -> bool:
        t_min = min(len(self.mdd), len(other.mdd))

        joint_mdd = {
            timestep: [] for timestep in range(t_min)
        }

        joint_mdd[0].append([self.mdd[0][0], other.mdd[0][0]])

        for t in range(t_min -1):
            for pair in joint_mdd[t]:
                children_1 = self.mdd[t + 1]
                children_2 = other.mdd[t + 1]
                for child_1 in children_1:
                    # Check if in parent-child relationship
                    if abs(child_1[0] - pair[0][0]) + abs(child_1[1] - pair[0][1]) == 1:
                        for child_2 in children_2:
                            if abs(child_2[0] - pair[1][0]) + abs(child_2[1] - pair[1][1]) == 1:
                                # Check vertex collision
                                if child_1 == child_2:
                                    return True
                                # Check edge collision
                                if child_1 == pair[1] and child_2 == pair[0]:
                                    return True
                                if child_1 != child_2 and [child_1, child_2] not in joint_mdd[t + 1]:
                                    joint_mdd[t + 1].append([child_1, child_2])

        # Check Dependency and cardinality for final
        if len(joint_mdd[t]) == 0:
            if len(joint_mdd[t - 1]) == 1:
                return True
        return False
