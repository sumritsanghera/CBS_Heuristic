import math
from single_agent_planner import move

def manhatten_distance(loc, goal):
    return abs(loc[0] - goal[0]) + abs(loc[1] - goal[1])

def optimal_paths(locs, goal) -> list:
    paths = dict()
    min_dist = math.inf
    for loc in locs:
        dist = manhatten_distance(loc, goal)
        if dist not in paths:
            paths[dist] = [loc]
        elif loc not in paths[dist]:
            paths[dist].append(loc)
        if dist < min_dist:
            min_dist = dist
    return paths[min_dist]


class MDD:
    def __init__(self, my_map, loc_start, loc_goal):
        self.loc_start = loc_start
        self.loc_goal = loc_goal
        self.my_map = my_map
        self.mdd = dict()
        self.generate_mdd()

    # construct an MDD for an agent with all the possible moves
    def generate_mdd(self):
    # Set root node in MDD to the start location of agent
        self.mdd[0] = [self.loc_start]
        time = 1
        while True:
            # print(f"Time: {time}") #:TEST
            possible_moves = []
            for loc in self.mdd[time - 1]:
                if loc == self.loc_goal:
                    # print(f"Goal reached at timestep {time}")  #TEST
                    print(f"check mdd: {self.mdd}")
                    return
                # Find possible moves that allow an agent to progress
                for i in range(4):
                    loc_next = move(loc, i)
                    # make sure all moves are within bounds
                    if (loc_next[0] < 0 or loc_next[0] >= len(self.my_map)
                    or loc_next[1] < 0 or loc_next[1] >= len(self.my_map[0])):
                        continue
                    # avoid adding obstacles
                    if self.my_map[loc_next[0]][loc_next[1]]:
                        continue
                    possible_moves.append(loc_next)
                    # print(f"Location: {location}, Possible Moves: {possible_moves}") #TEST
                self.mdd[time] = optimal_paths(possible_moves, self.loc_goal)


            # self.mdd[time] = possible_moves
            time += 1

    # Returns dependency and cardinality
    def is_dependent(self, other) -> bool:
        t_max = max(len(self.mdd), len(other.mdd))

        joint_mdd = {
            timestep: [] for timestep in range(t_max)
        }

        joint_mdd[0].append([self.mdd[0][0], other.mdd[0][0]])

        for t in range(t_max):
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
