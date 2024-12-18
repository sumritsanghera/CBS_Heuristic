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
                if d not in mdd1.mdd or d not in mdd2.mdd: 
                    continue
                if (len(mdd1.mdd[d]) == 1 and len(mdd2.mdd[d]) == 1 and 
                    mdd1.mdd[d][0] == mdd2.mdd[d][0]):
                    conflict_graph.add_node(i)
                    conflict_graph.add_node(j)
                    conflict_graph.add_edge(i, j)
                    cardinal_conflicts.append((i, j, d, mdd1.mdd[d][0]))
                    break

                #edge conflicts
                if d < depth - 2:  
                    if d+1 not in mdd1.mdd or d+1 not in mdd2.mdd: 
                        continue
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
def wdg_heuristic(mdd, agents, initial_paths, node, my_map, heuristics): 
    dependency_graph = networkx.Graph(name="Dependency Graph") 
    dependencies = []
    h_value = 0

    for i in range(agents-1): 
        for j in range(i+1, agents): 
            path1, path2 = get_paths(i, j, node, my_map, heuristics[i], heuristics[j])
            min_path_length = len(path1) + len(path2)
            cost_of_paths = len(initial_paths[i]) + len(initial_paths[j])
            diff = min_path_length - cost_of_paths
            if mdd[i].is_dependent(mdd[j]) and diff > 0:
                dependency_graph.add_edge(i, j, weight=diff)
                dependencies.append((i, j, diff))
    # Get edge-weighted min vertex cover
    if len(dependency_graph.edges) > 0: 
        connected_components = list(networkx.connected_components(dependency_graph))
        for component in connected_components: 
            # compute h_val as the minimum vertex_cover with min cost
            h_value += get_node_weights(dependency_graph, list(component))
    return h_value, dependencies 
