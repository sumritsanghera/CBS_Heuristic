# The file defines some helper functions for the heuristics
from scipy.optimize import linprog
import numpy as np
from single_agent_planner import build_constraint_table, a_star

# redefine standard_splitting for collisions from CBS: 
def standard_splitting(collision):
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

# get node weights for weighted dependency graph
def get_node_weights(dependency_graph, component):
    # only one edge
    if len(component) == 2: 
        return dependency_graph[component[0]][component[1]]['weight']

    # get minimum vertex weights by solving a system of linear equations 
    # between node weights and their edge weights
    else: 
        num_vertices = len(component)
        num_edges = 0
        edge_weights = []
        edges = []
        vertex_indices = dict()

        # map agent_id's to indices for solving node_weights
        ind = 0
        for vertex in component: 
            vertex_indices[vertex] = ind
            ind+=1

        # get edges, and edge_weights
        for u, v, weight in dependency_graph.edges(data=True): 
            if u in component and v in component: 
                num_edges+=1
                edge_weights.append(-weight['weight'])
                edges.append((u, v))

        coefficients = [0]*num_vertices
        bounds = [(0, None)]*num_vertices

        # set A matrix to solve the system of linear equations
        A = np.zeros((num_edges, num_vertices))
        for i in range(num_edges): 
            agent1 = edges[i][0]
            agent2 = edges[i][1]
            A[i][vertex_indices[agent1]] = -1
            A[i][vertex_indices[agent2]] = -1

        # get the node weights that minimize the sum of node weights for edge-weighted MVC
        result = linprog(coefficients, A_ub=A, b_ub=edge_weights, bounds=bounds, method="highs") 
        if (result.success): 
            return int(sum(result.x))
        return 0

# find conflict free paths between two agents to get edge weights
def get_paths(agent1, agent2, node, my_map, agent1_heuristics, agent2_heuristics): 
    # find constraints between colliding agents only
    constraints = []
    for collision in node['collisions']: 
        if collision['a1'] == agent1 and collision['a2'] == agent2: 
            both_constraints = standard_splitting(collision)
            for constraint in both_constraints: 
                constraints.append(constraint)

    agent1_start_loc = node['paths'][agent1][0]
    agent2_start_loc = node['paths'][agent2][0]

    agent1_end_loc = node['paths'][agent1][-1]
    agent2_end_loc = node['paths'][agent2][-1]

    path1 = a_star(my_map, agent1_start_loc, agent1_end_loc, agent1_heuristics, agent1, constraints)
    path2 = a_star(my_map, agent2_start_loc, agent2_end_loc, agent2_heuristics, agent2, constraints)
    return path1, path2

