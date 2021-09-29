"""
Contains functions related to calculating the MST of a graph
"""
import heapq

# Function to calculate the cost of the Minimum Spanning Tree of the given graph, such that the MST only includes vertices from the 'allowed_node_list'
def MST(graph, allowed_node_list):
    # Currently using an N^2 implementation for testing, can be optimized further
    mst_costs = {}
    included_in_mst = {}
    final_heuristic = 0
    nodes_included = 0
    required_num_nodes = len(allowed_node_list) 
    for key in allowed_node_list:
        mst_costs[key] = None
        included_in_mst[key] = 0

    # Pick a random vertex
    if len(allowed_node_list) > 0:
        node = allowed_node_list[0]
        mst_costs[node] = 0
        
        while nodes_included < required_num_nodes:
            # Get the min value node not yet included
            min_cost_node = None
            min_yet = None
            for key in allowed_node_list:
                
                if included_in_mst[key] == 0:
                    if mst_costs[key] is not None:
                        if min_yet == None or min_yet > mst_costs[key]:
                            min_cost_node = key
                            min_yet = mst_costs[key]
            
            if min_cost_node is not None:
                # MST can be formed
                included_in_mst[min_cost_node] = 1
                nodes_included += 1
                final_heuristic += min_yet
                # Update the costs of all adjacent nodes
                for key in allowed_node_list:
                    if included_in_mst[key] == 0:
                        if graph[min_cost_node].get(key) is not None:
                            if mst_costs[key]==None or graph[min_cost_node][key]<mst_costs[key]:
                                mst_costs[key] = graph[min_cost_node][key]
            else:
                # MST does not exist
                return False
    return final_heuristic

def MST_with_heaps(graph, allowed_node_list):
    # O(Nlog(N)) using heap for prim's mst
    nodes = graph.keys()
    visited = set()
    allowed = set(allowed_node_list)
    mst_cost = 0
    heap = []
    if len(allowed_node_list) > 0:
        node = allowed_node_list[0]
        heapq.heappush(heap,(0,node))

        while heap:
            cost, current = heapq.heappop(heap)
            if current not in visited:
                visited.add(current)
                mst_cost += cost

            for neighbor in graph[current].keys():
                if neighbor not in visited and neighbor in allowed:
                    heapq.heappush(heap,(graph[current][neighbor],neighbor))
        if len(visited) != len(allowed_node_list):
            return False

    return mst_cost