import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, build_constraint_table
from paths_violate_constraint import paths_violate_constraint
from mdd import *
from conflict_graph import *


def detect_collision(path1, path2):
    collisions = []
    max_timestep = max(len(path1), len(path2))
    
    for t in range(max_timestep):
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)
        
        if loc1 == loc2:
            collisions.append({'a1': path1, 'a2': path2, 'loc': [loc1], 'timestep': t})
        
        if t < max_timestep - 1:
            next_loc1 = get_location(path1, t + 1)
            next_loc2 = get_location(path2, t + 1)
            if loc1 == next_loc2 and loc2 == next_loc1:
                collisions.append({'a1': path1, 'a2': path2, 'loc': [loc1, loc2], 'timestep': t + 1})
    
    return collisions

def detect_collisions(paths):
    all_collisions = []
    num_agents = len(paths)
    
    for i in range(num_agents):
        for j in range(i + 1, num_agents):
            collisions = detect_collision(paths[i], paths[j])
            for collision in collisions:
                all_collisions.append({
                    'a1': i,
                    'a2': j,
                    'loc': collision['loc'],
                    'timestep': collision['timestep']
                })
    return all_collisions

def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    #pass

    constraints = []
    
    if len(collision['loc']) == 1:
        #vertex collision
        constraints.append({
            'agent': collision['a1'], 
            'loc': collision['loc'], 
            'timestep': collision['timestep']
        })
        constraints.append({
            'agent': collision['a2'], 
            'loc': collision['loc'], 
            'timestep': collision['timestep']
        })
    else:
        #edge collision
        constraints.append({
            'agent': collision['a1'], 
            'loc': [collision['loc'][0], collision['loc'][1]], 
            'timestep': collision['timestep']
        })
        constraints.append({
            'agent': collision['a2'], 
            'loc': [collision['loc'][1], collision['loc'][0]], 
            'timestep': collision['timestep']
        })

    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    #pass

    #randomly choose one agents in collision
    if random.randint(0, 1) == 0:
        this_agent = collision['a1']
        other_agent = collision['a2']
    elif random.randint(0, 1) == 1:
        this_agent = collision['a2']
        other_agent = collision['a1']

    constraints = []
    
    if len(collision['loc']) == 1:
        #vertex collision
        constraints.append({
            'agent': this_agent, 
            'loc': collision['loc'], 
            'timestep': collision['timestep'],
            'positive': True  #positive constraint for this agent
        })
        constraints.append({
            'agent': other_agent, 
            'loc': collision['loc'], 
            'timestep': collision['timestep'],
            'positive': False  #negative constraint for other agent
        })
    else:
        #edge collision
        constraints.append({
            'agent': this_agent, 
            'loc': [collision['loc'][0], collision['loc'][1]], 
            'timestep': collision['timestep'],
            'positive': True  #positive constraint for this agent
        })
        constraints.append({
            'agent': other_agent, 
            'loc': [collision['loc'][1], collision['loc'][0]], 
            'timestep': collision['timestep'],
            'positive': False  #negative constraint for other agent
        })

    return constraints


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals, heuristic_option=0):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        heuristic_option - 0: CG heuristic, 1: DG heuristic, -1: No heuristic
        """
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.heuristic_option = heuristic_option
        self.final_dependencies = set()
        self.final_conflicts = set()
        self.open_list = []

        self.mdds = dict()
        self.initial_paths = []
        
        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    # def build_mdds(self, node, paths):
    #     mdds = []
    #     for agent in range(self.num_of_agents):
    #         path_cost = len(paths[agent]) - 1
    #         mdd = MDD(self.my_map, self.starts[agent], self.goals[agent],
    #                   self.heuristics[agent], agent, node['constraints'], path_cost)
    #         mdds.append(mdd)
    #     return mdds
    def build_mdds(self): #updated for new mdd.py
        for agent in range(self.num_of_agents):
            self.mdds[agent] = MDD(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], [], [])

    def update_mdd(self, agent, constraints, paths): 
        self.mdds[agent].update_mdd(agent, constraints, paths)


    def classify_collision(self, collision):
        """Classifies the collisions to help debug during runtime"""
        print("==COLLISION!==")
        print(f"Location(s): {collision['loc']}")
        print(f"Timestep: {collision['timestep']}")
        #collision_type = detect_cardinal_conflicts(collision, mdds)
        #print(f"Collision classified as: {collision_type}")
        #return collision_type

    def push_node(self, node):
        h_val = 0
        if self.heuristic_option == 0:
            conflicts = compute_cg_heuristic(self.mdds, self.num_of_agents)
            self.final_conflicts.update(conflicts)

        elif self.heuristic_option == 1:
            h_val, dependencies = compute_dg_heuristic(self.mdds, self.num_of_agents)
            self.final_dependencies.update(dependencies)

        elif self.heuristic_option == 2:
            h_val, dependencies = compute_wdg_heuristic(self.mdds, self.num_of_agents, self.initial_paths, node['paths'])
            self.final_dependencies.update(dependencies)

        #calculate the f-value
        f_val = node['cost'] + h_val
        node['h_val'] = h_val #store h-value in node

        heapq.heappush(self.open_list, (f_val, h_val, len(node['collisions']), self.num_of_generated, node))

        #print("Generate node {} with f-val {} (g-val {} + h-val {})".format(self.num_of_generated, f_val, node['cost'], h_val))
        #heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated, h_val))
        self.num_of_generated += 1
        

    # def pop_node(self):
    #     _, _, id, node = heapq.heappop(self.open_list)
    #     print("Expand node {}".format(id))
    #     self.num_of_expanded += 1
    #     return node
    def pop_node(self): #update function to accomodate changed push_node
        _, _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {} with f-val {} (g-val {} + h-val {})".format(
            id, node['cost'] + node['h_val'], node['cost'], node['h_val']))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        # Generate the root node
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [],
                'mdds': None}  # Will store MDDs for conflict classification

        #find initial paths for each agent
        for i in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                         i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        self.initial_paths = root['paths']

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        # build MDD for all agents
        self.build_mdds()
        
        self.push_node(root)

        while len(self.open_list) > 0:
            P = self.pop_node()
            
            if len(P['collisions']) == 0:
                self.print_results(P)
                return P['paths']

            # Choose collision and classify it
            collision = P['collisions'][0]
            collision_type = self.classify_collision(collision)

            constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)

            for constraint in constraints:
                Q = {
                    'constraints': P['constraints'] + [constraint],
                    'paths': P['paths'].copy(),
                    'collisions': [],
                    'cost': 0,
                    'mdds': None
                }

                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                             self.heuristics[agent], agent, Q['constraints'])
                
                if path is not None:
                    Q['paths'][agent] = path
                    if disjoint and constraint['positive']:
                        violating_agents = paths_violate_constraint(constraint, Q['paths'])
                        all_paths_valid = True
                        for v_agent in violating_agents:
                            if v_agent != agent:
                                v_constraint = {
                                    'agent': v_agent,
                                    'loc': constraint['loc'],
                                    'timestep': constraint['timestep'],
                                    'positive': False
                                }
                                Q['constraints'].append(v_constraint)
                                
                                v_path = a_star(self.my_map, self.starts[v_agent],
                                               self.goals[v_agent], self.heuristics[v_agent],
                                               v_agent, Q['constraints'])
                                
                                if v_path is None:
                                    all_paths_valid = False
                                    break
                                Q['paths'][v_agent] = v_path
                        if not all_paths_valid:
                            continue
                    
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    self.update_mdd(agent, Q['constraints'], Q['paths'][agent])
                    self.push_node(Q)
        self.print_results(root)
        return root['paths']

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
       
        if self.heuristic_option == 1:
            print("\nDependency Summary:")
            print(f"Total number of dependencies found: {len(self.final_dependencies)}")
            print("Dependencies between agents:")
            for i, j in sorted(self.final_dependencies):
                print(f"  Agents {i} and {j}")
            final_graph = networkx.Graph()
            for i, j in self.final_dependencies:
                final_graph.add_edge(i, j)
        
            mvc_size = len(vertex_cover.min_weighted_vertex_cover(final_graph))
            print(f"\nFinal dependency graph heuristic value (H_dg): {mvc_size}")
        
        elif self.heuristic_option == 0:
            print("\nCardinal Conflict Summary")
            print(f"Total number of cardinal conflicts found: {len(self.final_conflicts)}")
            print("Cardinal conflicts between agents:")
            final_graph = networkx.Graph()
            for i, j, d, loc in self.final_conflicts:
                final_graph.add_edge(i, j)
                print(f"  Agents {i} and {j} at depth {d}, location: {loc}")
                
            mvc_size = len(vertex_cover.min_weighted_vertex_cover(final_graph))
            print(f"\nFinal conflict graph heuristic value (H_cg): {mvc_size}")
            
