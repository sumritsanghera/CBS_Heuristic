# import networkx
# from networkx.algorithms.approximation import vertex_cover
# from mdd import *

# def compute_cg_heuristic(mdds, agents):
#     """
#     Compute h-value using the conflict graph heuristic based on cardinal conflicts
#     """
#     conflict_graph = networkx.Graph(name="Conflict Graph")
#     cardinal_conflicts = []
    
#     # Check each pair of agents
#     for i in range(agents - 1):
#         for j in range(i + 1, agents):
#             mdd1 = mdds[i].mdd
#             mdd2 = mdds[j].mdd
            
#             # Find depth for comparison
#             depth = min(len(mdd1), len(mdd2))
            
#             # Check each level for cardinal conflicts
#             for d in range(depth):
#                 if (len(mdd1[d]) == 1 and len(mdd2[d]) == 1 and 
#                     mdd1[d][0] == mdd2[d][0]):
#                     conflict_graph.add_node(i)
#                     conflict_graph.add_node(j)
#                     conflict_graph.add_edge(i, j)
#                     cardinal_conflicts.append((i, j, d, mdd1[d][0]))
#                     break
                
#                 # Check for cardinal edge conflicts if not at last level
#                 if d < depth - 1:
#                     # Only check if both agents have exactly one possible move
#                     if len(mdd1[d]) == 1 and len(mdd2[d]) == 1:
#                         loc1 = mdd1[d][0]
#                         loc2 = mdd2[d][0]
                        
#                         # Get next possible locations
#                         next_locs1 = mdds[i].mdd_edges[d].get(loc1, [])
#                         next_locs2 = mdds[j].mdd_edges[d].get(loc2, [])
                        
#                         # Cardinal edge conflict if agents must swap positions
#                         if (len(next_locs1) == 1 and len(next_locs2) == 1 and
#                             next_locs1[0] == loc2 and next_locs2[0] == loc1):
#                             conflict_graph.add_node(i)
#                             conflict_graph.add_node(j)
#                             conflict_graph.add_edge(i, j)
#                             cardinal_conflicts.append((i, j, d, (loc1, loc2)))
#                             break
    
#     # Compute and print h-value
#     h_value = 0
#     if len(conflict_graph.edges) > 0:
#         h_value = len(vertex_cover.min_weighted_vertex_cover(conflict_graph))
    
#     print("CG Heuristic Calculation:")
#     print(f"Number of cardinal conflicts found: {len(cardinal_conflicts)}")
#     for i, j, d, loc in cardinal_conflicts:
#         print(f"Cardinal conflict between agents {i} and {j} at depth {d}, location: {loc}")
#     print(f"Final CG heuristic value (Hcg): {h_value}\n")
    
#     return h_value

# def print_cg_graph(mdds, agents):
#     """
#     Helper function to print the conflict graph for debugging
#     """
#     conflict_graph = networkx.Graph(name="Conflict Graph")
    
#     print("\nBuilding Conflict Graph:")
#     for i in range(agents - 1):
#         for j in range(i + 1, agents):
#             mdd1 = mdds[i].mdd
#             mdd2 = mdds[j].mdd
#             depth = min(len(mdd1), len(mdd2))
            
#             for d in range(depth):
#                 if len(mdd1[d]) == 1 and len(mdd2[d]) == 1:
#                     if mdd1[d][0] == mdd2[d][0]:
#                         print(f"Cardinal conflict between agents {i} and {j} at depth {d}")
#                         print(f"Location: {mdd1[d][0]}")
#                         conflict_graph.add_node(i)
#                         conflict_graph.add_node(j)
#                         conflict_graph.add_edge(i, j)
#                         break
    
#     print("\nConflict Graph:")
#     print(f"Nodes: {list(conflict_graph.nodes)}")
#     print(f"Edges: {list(conflict_graph.edges)}")
#     h_value = len(vertex_cover.min_weighted_vertex_cover(conflict_graph))
#     print(f"Heuristic value (MVC size): {h_value}\n")

    ###############
import networkx
from networkx.algorithms.approximation import vertex_cover
from mdd import *

def compute_cg_heuristic(mdds, agents):
    """
    Compute h-value using the conflict graph heuristic based on cardinal conflicts.
    The h-value is the number of unique agent pairs that have cardinal conflicts.
    """
    # Keep track of agent pairs that have cardinal conflicts
    cardinal_pairs = set()
    
    # Check each pair of agents
    for i in range(agents - 1):
        for j in range(i + 1, agents):
            mdd1 = mdds[i].mdd
            mdd2 = mdds[j].mdd
            
            # Find depth for comparison
            depth = min(len(mdd1), len(mdd2))
            has_cardinal = False
            
            # Check each level for cardinal conflicts
            for d in range(depth):
                # Cardinal vertex conflict exists if both agents have only one node 
                # at this level and it's the same node
                if (len(mdd1[d]) == 1 and len(mdd2[d]) == 1 and 
                    mdd1[d][0] == mdd2[d][0]):
                    has_cardinal = True
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
                            has_cardinal = True
                            break
            
            # If a cardinal conflict was found between these agents, add to set
            if has_cardinal:
                cardinal_pairs.add((i, j))
    
    # Print detailed information about cardinal conflicts
    print("\nCG Heuristic Calculation:")
    print(f"Cardinal conflicts found between agent pairs: {sorted(cardinal_pairs)}")
    h_value = len(cardinal_pairs)
    print(f"Final CG heuristic value (Hcg): {h_value} (number of agent pairs with cardinal conflicts)\n")
    
    return h_value

def return_optimal_conflict(conflicts, mdds):
    """Modified to work with MDD class and count cardinal conflicts"""
    semi_cardinals = []
    non_cardinals = []
    cardinal_conflicts = []
    
    for conflict in conflicts:
        mdd1 = mdds[conflict['a1']].mdd
        mdd2 = mdds[conflict['a2']].mdd
        location = conflict['loc']
        timestep = conflict['timestep']

        # One agent already reaches its goal
        if timestep >= len(mdd1) or timestep >= len(mdd2):
            non_cardinals.append(conflict)
            continue
            
        # Cardinal conflict - vertex
        if len(conflict['loc']) == 1:
            loc = location[0]
            if (len(mdd1[timestep]) == 1 and len(mdd2[timestep]) == 1 and 
                mdd1[timestep][0] == loc and mdd2[timestep][0] == loc):
                cardinal_conflicts.append(conflict)
                
            # Semi-cardinal conflict
            elif ((len(mdd1[timestep]) == 1 or len(mdd2[timestep]) == 1) and 
                  loc in mdd1[timestep] and loc in mdd2[timestep]):
                semi_cardinals.append(conflict)
                
            # Non cardinal conflict
            else:
                non_cardinals.append(conflict)
                
        # Edge conflict
        else:
            loc1, loc2 = location[0], location[1]
            prev_timestep = timestep - 1
            
            # Check if edge exists in both MDDs
            edge1_exists = False
            edge2_exists = False
            
            if prev_timestep in mdds[conflict['a1']].mdd_edges:
                if loc1 in mdds[conflict['a1']].mdd_edges[prev_timestep]:
                    edge1_exists = loc2 in mdds[conflict['a1']].mdd_edges[prev_timestep][loc1]
                    
            if prev_timestep in mdds[conflict['a2']].mdd_edges:
                if loc2 in mdds[conflict['a2']].mdd_edges[prev_timestep]:
                    edge2_exists = loc1 in mdds[conflict['a2']].mdd_edges[prev_timestep][loc2]
            
            if edge1_exists and edge2_exists:
                if len(mdd1[timestep]) == 1 and len(mdd2[timestep]) == 1:
                    cardinal_conflicts.append(conflict)
                elif len(mdd1[timestep]) == 1 or len(mdd2[timestep]) == 1:
                    semi_cardinals.append(conflict)
            else:
                non_cardinals.append(conflict)
    
    # Print cardinal conflict information
    print(f"\nNumber of cardinal conflicts: {len(cardinal_conflicts)}")
    for conflict in cardinal_conflicts:
        print(f"Cardinal conflict between agents {conflict['a1']} and {conflict['a2']} at location {conflict['loc']}, timestep {conflict['timestep']}")
    
    # Return first cardinal conflict if any exist, otherwise follow original logic
    if len(cardinal_conflicts) > 0:
        return cardinal_conflicts[0], len(cardinal_conflicts)
    if len(semi_cardinals) > 0:
        return semi_cardinals[0], 0
    return non_cardinals[0], 0