import networkx
from networkx.algorithms.approximation import vertex_cover
from mdd import *
from helper import *

def cg_heuristic(mdd, agents):
    """Compute H_cg using the conflict graph heuristic based on cardinal conflicts"""
    conflict_graph = networkx.Graph(name="Conflict Graph")
    cardinal_conflicts = []
    
    #check each pair
    for i in range(agents - 1):
        for j in range(i + 1, agents):
            mdd1 = mdd[i]  
            mdd2 = mdd[j] 
            
            #find depth for comparison
            depth = min(len(mdd1.mdd), len(mdd2.mdd))
            
            #check each level for cardinal conflicts
            for d in range(depth):
                #vertex conflicts
                if (len(mdd1.mdd[d]) == 1 and len(mdd2.mdd[d]) == 1 and 
                    mdd1.mdd[d][0] == mdd2.mdd[d][0]):
                    conflict_graph.add_node(i)
                    conflict_graph.add_node(j)
                    conflict_graph.add_edge(i, j)
                    cardinal_conflicts.append((i, j, d, mdd1.mdd[d][0]))
                    break

                #edge conflicts
                if d < depth - 1:  
                    curr1 = mdd1.mdd[d][0]
                    curr2 = mdd2.mdd[d][0]
                    next1 = mdd1.mdd[d+1][0]
                    next2 = mdd2.mdd[d+1][0]
                    
                    #if agents swap positions -- needed for mdd_test3
                    if curr1 == next2 and curr2 == next1:
                        conflict_graph.add_node(i)
                        conflict_graph.add_node(j)
                        conflict_graph.add_edge(i, j)
                        cardinal_conflicts.append((i, j, d, (curr1, curr2)))
                        break
    h_val = 0
    if len(conflict_graph.edges) > 0:
        h_val = len(vertex_cover.min_weighted_vertex_cover(conflict_graph))

    return cardinal_conflicts, h_val

def dg_heuristic(mdd, agents):
    """Compute H_dg value based on dependencies between agents"""
    dependency_graph = networkx.Graph(name="Dependency Graph")
    dependencies = []
    
    #check each pair of agents for dependencies
    for i in range(agents - 1):
        for j in range(i + 1, agents):
            #is_dependent method from mdd.py
            if mdd[i].is_dependent(mdd[j]):
                dependency_graph.add_node(i)
                dependency_graph.add_node(j)
                dependency_graph.add_edge(i, j)
                dependencies.append((i, j))
    
    #compute h-value using minimum vertex cover
    h_value = 0
    if len(dependency_graph.edges) > 0:
        h_value = len(vertex_cover.min_weighted_vertex_cover(dependency_graph))
    return h_value, dependencies

# WDG heuristic
import networkx
from networkx.algorithms.approximation import vertex_cover
from mdd import *
from scipy.optimize import linprog
import numpy as np


def compute_cg_heuristic(mdd, agents):
    """Compute h-value using the conflict graph heuristic based on cardinal conflicts"""
    conflict_graph = networkx.Graph(name="Conflict Graph")
    cardinal_conflicts = []
    
    #check each pair of agents
    for i in range(agents - 1):
        for j in range(i + 1, agents):
            mdd1 = mdd[i]  
            mdd2 = mdd[j] 
            
            #find depth for comparison
            depth = min(len(mdd1.mdd), len(mdd2.mdd))
            
            #check each level for cardinal conflicts
            for d in range(depth):
                #vertex conflicts
                if (len(mdd1.mdd[d]) == 1 and len(mdd2.mdd[d]) == 1 and 
                    mdd1.mdd[d][0] == mdd2.mdd[d][0]):
                    conflict_graph.add_node(i)
                    conflict_graph.add_node(j)
                    conflict_graph.add_edge(i, j)
                    cardinal_conflicts.append((i, j, d, mdd1.mdd[d][0]))
                    break

                #edge conflicts
                if d < depth - 1:  
                    curr1 = mdd1.mdd[d][0]
                    curr2 = mdd2.mdd[d][0]
                    next1 = mdd1.mdd[d+1][0]
                    next2 = mdd2.mdd[d+1][0]
                    
                    #if agents swap positions -- needed for mdd_test3
                    if curr1 == next2 and curr2 == next1:
                        conflict_graph.add_node(i)
                        conflict_graph.add_node(j)
                        conflict_graph.add_edge(i, j)
                        cardinal_conflicts.append((i, j, d, (curr1, curr2)))
                        break
    h_val = 0
    if len(conflict_graph.edges) > 0:
        h_val = len(vertex_cover.min_weighted_vertex_cover(conflict_graph))

    return cardinal_conflicts, h_val

def compute_dg_heuristic(mdd, agents):
    """Compute H_dg value based on dependencies between agents"""
    dependency_graph = networkx.Graph(name="Dependency Graph")
    dependencies = []
    
    #check each pair of agents for dependencies
    for i in range(agents - 1):
        for j in range(i + 1, agents):
            #is_dependent method from mdd.py
            if mdd[i].is_dependent(mdd[j]):
                dependency_graph.add_node(i)
                dependency_graph.add_node(j)
                dependency_graph.add_edge(i, j)
                dependencies.append((i, j))
    
    #compute h-value using minimum vertex cover
    h_value = 0
    if len(dependency_graph.edges) > 0:
        h_value = len(vertex_cover.min_weighted_vertex_cover(dependency_graph))
    return h_value, dependencies

def get_node_weights(dependency_graph, component):
    # Single edge case
    if len(component) == 2:
        u, v = component
        return dependency_graph[u][v]['weight']
    
    # For triangles/components
    edge_weights = []
    for u, v in dependency_graph.edges():
        if u in component and v in component:
            edge_weights.append(dependency_graph[u][v]['weight'])
            
    # Take the sum of the two largest weights divided by 2
    edge_weights.sort(reverse=True)  # Sort in descending order
    max_weight_sum = edge_weights[0] + edge_weights[1] if len(edge_weights) > 1 else edge_weights[0]
    return max_weight_sum // 2
    
def wdg_heuristic(mdd, agents, initial_paths, paths):
    dependency_graph = networkx.Graph(name="Dependency Graph")
    edge_weights = dict()
    dependencies = []
    h_value = 0
    
    # Find dependencies and their weights
    for i in range(agents-1):
        for j in range(i+1, agents):
            if mdd[i].is_dependent(mdd[j]):
                # Let's ensure we're getting correct weight calculation
                initial_cost = len(initial_paths[i]) + len(initial_paths[j]) - 2  # -2 because paths include start position
                new_cost = len(paths[i]) + len(paths[j]) - 2
                weight = max(1, new_cost - initial_cost)  # ensure minimum weight of 1
                
                dependency_graph.add_edge(i, j, weight=weight)
                dependencies.append((i, j))
                edge_weights[(i, j)] = weight
                print(f"Debug: Edge ({i},{j}) has weight {weight}")  # Debug print
    
    # Calculate h-value from components
    if dependency_graph.edges:
        for component in networkx.connected_components(dependency_graph):
            component_weight = get_node_weights(dependency_graph, list(component))
            h_value += component_weight
            print(f"Debug: Component {list(component)} contributes {component_weight}")  # Debug print
    
    return h_value, dependencies, edge_weights





