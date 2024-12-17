import networkx
from networkx.algorithms.approximation import vertex_cover
from mdd import *

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

    return cardinal_conflicts

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
    # only one edge
    if len(component) == 2: 
        return dependency_graph[component[0]][component[1]]['weight']
    else: 
        vertex_weights = dict()
        for i in range(len(component)-1): 
            for j in range(i+1, len(component)): 
                ## TODO return the correct sum of node weights corresponding to the edge weights
                print("the weight is", dependency_graph[component[i]][component[j]]['weight'])
                return 0

# WDG heuristic
def compute_wdg_heuristic(mdd, agents, initial_paths, paths): 
    dependency_graph = networkx.Graph(name="Dependency Graph") 
    edge_weights = [] 
    dependencies = []
    h_value = 0
    for i in range(agents-1): 
        for j in range(i+1, agents): 
            min_path_length = len(paths[i]) + len(paths[j])
            cost_of_paths = len(initial_paths[i]) + len(initial_paths[j])
            diff = min_path_length - cost_of_paths
            if mdd[i].is_dependent(mdd[j]) and diff > 0:
                dependency_graph.add_edge(i, j, weight=diff)
                dependencies.append((i, j))
        ## TODO get edge-weighted minimum vertex cover
    if len(dependency_graph.edges) > 0: 
        print("the min vertex cover is", vertex_cover.min_weighted_vertex_cover(dependency_graph))
    for u, v, weight in dependency_graph.edges(data=True):
        print(u, v, weight)
    connected_components = list(networkx.connected_components(dependency_graph))
    for component in connected_components: 
        h_value += get_node_weights(dependency_graph, list(component))
    print("the connected commpontnets are", connected_components)
    return h_value, dependencies
