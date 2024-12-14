import networkx
from networkx.algorithms.approximation import vertex_cover
from mdd import *

def compute_cg_heuristic(mdds, agents):
    """
    Compute h-value using the conflict graph heuristic based on cardinal conflicts
    """
    conflict_graph = networkx.Graph(name="Conflict Graph")
    cardinal_conflicts = []
    
    # Check each pair of agents
    for i in range(agents - 1):
        for j in range(i + 1, agents):
            mdd1 = mdds[i].mdd
            mdd2 = mdds[j].mdd
            
            # Find depth for comparison
            depth = min(len(mdd1), len(mdd2))
            
            # Check each level for cardinal conflicts
            for d in range(depth):
                if (len(mdd1[d]) == 1 and len(mdd2[d]) == 1 and 
                    mdd1[d][0] == mdd2[d][0]):
                    conflict_graph.add_node(i)
                    conflict_graph.add_node(j)
                    conflict_graph.add_edge(i, j)
                    cardinal_conflicts.append((i, j, d, mdd1[d][0]))
                    break
                
                # Check for cardinal edge conflicts if not at last level
                if d < depth - 1:
                    # Only check if both agents have exactly one possible move
                    if len(mdd1[d]) == 1 and len(mdd2[d]) == 1:
                        loc1 = mdd1[d][0]
                        loc2 = mdd2[d][0]
                        
                        # Get next possible locations
                        next_locs1 = mdds[i].mdd_edges[d].get(loc1, [])
                        next_locs2 = mdds[j].mdd_edges[d].get(loc2, [])
                        
                        # Cardinal edge conflict if agents must swap positions
                        if (len(next_locs1) == 1 and len(next_locs2) == 1 and
                            next_locs1[0] == loc2 and next_locs2[0] == loc1):
                            conflict_graph.add_node(i)
                            conflict_graph.add_node(j)
                            conflict_graph.add_edge(i, j)
                            cardinal_conflicts.append((i, j, d, (loc1, loc2)))
                            break
    
    # Compute and print h-value
    h_value = 0
    if len(conflict_graph.edges) > 0:
        h_value = len(vertex_cover.min_weighted_vertex_cover(conflict_graph))
    
    for i, j, d, loc in cardinal_conflicts:
        print(f"Cardinal conflict between agents {i} and {j} at depth {d}, location: {loc}")    
    return h_value

def print_cg_graph(mdds, agents):
    """
    Helper function to print the conflict graph for debugging
    """
    conflict_graph = networkx.Graph(name="Conflict Graph")
    
    print("\nBuilding Conflict Graph:")
    for i in range(agents - 1):
        for j in range(i + 1, agents):
            mdd1 = mdds[i].mdd
            mdd2 = mdds[j].mdd
            depth = min(len(mdd1), len(mdd2))
            
            for d in range(depth):
                if len(mdd1[d]) == 1 and len(mdd2[d]) == 1:
                    if mdd1[d][0] == mdd2[d][0]:
                        print(f"Cardinal conflict between agents {i} and {j} at depth {d}")
                        print(f"Location: {mdd1[d][0]}")
                        conflict_graph.add_node(i)
                        conflict_graph.add_node(j)
                        conflict_graph.add_edge(i, j)
                        break
    
    print(f"Nodes: {list(conflict_graph.nodes)}")
    print(f"Edges: {list(conflict_graph.edges)}")
    h_value = len(vertex_cover.min_weighted_vertex_cover(conflict_graph))
    print(f"Heuristic value (MVC size): {h_value}\n")

    ###############

    