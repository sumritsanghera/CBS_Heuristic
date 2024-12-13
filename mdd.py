from single_agent_planner import *
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
        constraint_table = build_constraint_table(self.constraints, self.agent)
        
        root_node = {'prev': None, 'cost': 0, 'curr_loc': self.start_loc}
        open_list = collections.deque([root_node])
        
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