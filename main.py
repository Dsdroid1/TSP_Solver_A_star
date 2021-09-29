"""
AI Assignment 2 by BT18CSE046
TSP Solver using A* with MST as the heuristic
"""
import heapq
from mst import MST_with_heaps
from state import state

# Function to read the graph as an input file
def read_input(filename):
    """
    The format of the input file should be:
    (i) The first line should contain the number of nodes in the graph(will be numbered from 1 to n)
    (ii) The subsequent lines contain info about the edges of the graph, in the format:
        i j cost or i,j,cost if the file extension is csv
        where i and j represent the nodes involved in the edge and cost represents the path coset for this edge
    """
    with open(filename, 'r') as file:
        lines = file.readlines()
        no_of_nodes = int(lines.pop(0).strip())
        
        # Create the adjacency list using dictionary
        graph = {}
        for i in range(1,no_of_nodes+1):
            graph[i] = {}

        separator = " "
        dot_pose = filename.find('.')
        if dot_pose != -1:
            file_extension = filename[dot_pose+1:]
            if file_extension == 'csv':
                separator = ','

        for line in lines:
            i,j,cost = list(map(int,line.strip().split(separator)))
            if i!=j:
                # Assuming undirected graph
                graph[i][j] = cost
                graph[j][i] = cost
        
        return graph 

def heuristic_function(graph, remaining_nodes, current_node, source):
    # The heuristic value is MST + path cost to closest node unvisited till now from the recent node + path cost of closest unvisited node from source
    # Note: This heuristic is more "refined", and hence will dominate the simple mst cost
    nodes = [key for key in graph.keys()]
    # mst_cost = MST(graph,remaining_nodes)
    mst_cost = MST_with_heaps(graph,remaining_nodes)
    nodes_remaining = set(remaining_nodes)
    if mst_cost is not False:
        if len(remaining_nodes) != 0:
            current_node_to_closest_unvisited_node_cost = None
            closest_unvisited_node_to_source_vertex_cost = None
            # Get the edge cost of visiting the closest node in the mst from the current vertex(i.e., the one considered as the next candidate )
            for key in graph[current_node].keys():
                if key in nodes_remaining:
                    if current_node_to_closest_unvisited_node_cost == None or current_node_to_closest_unvisited_node_cost > graph[current_node][key]:
                        current_node_to_closest_unvisited_node_cost = graph[current_node][key]

            # Get the edge cost of visiting the source vertex from the closest node in the mst, to complete the tour
            for key in graph[source].keys():
                if key in nodes_remaining:
                    if closest_unvisited_node_to_source_vertex_cost == None or closest_unvisited_node_to_source_vertex_cost > graph[source][key]:
                        closest_unvisited_node_to_source_vertex_cost = graph[source][key]
            
            if closest_unvisited_node_to_source_vertex_cost != None and current_node_to_closest_unvisited_node_cost != None:
                return closest_unvisited_node_to_source_vertex_cost+mst_cost+current_node_to_closest_unvisited_node_cost
            else:
                # Tour cant be completed, then no use in considering the current node to be added
                return False     
        else:
            # No other nodes will be left(i.e. MST = single node), we need to pick the actual edge cost as the heuristic estimate
            edge_cost = graph[current_node].get(source)
            if edge_cost is not None:
                return edge_cost
            else:
                return False

    else:
        # MST does not exist, hence tour cant be completed as the relaxed version of the problem itself cant be solved
        return False

def print_current_path(state):
    # Print the path taken to reach the state
    # Used only to show the progress of A*
    path = state.get_path_order()
    path_string = ""
    for node in path:
        path_string += f"-{node}"
    return path_string

def Successor_function(current_state, graph, source):
    successors = []
    # In successors, we store info about node, action cost, heuristic,action, already_visited_nodes (state data structure)
    if len(current_state.get_already_visited_nodes()) == len(graph.keys()):
        # All nodes visited once, means we now only need to connect to source vertex
        if graph[current_state.get_node()].get(source) is not None:
            target = source
            successor = state(target,None,None,None,None) # Create a new state
            # Set the g-value of this state appropriately
            successor.set_action_cost(current_state.get_action_cost() + graph[current_state.get_node()][target])
            # Since we know that this is one of the goal states, set the heuristic as 0
            successor.set_heuristic(0)
            # Update the path order(Not required for final solution, just to display the progress of A*)
            successor.set_path_order(current_state.get_path_order())
            successor.append_path_order(target)
            # Set the action from parent, used to create the final solution
            successor.set_action(f'{current_state.get_node()}-{source}')
            # Update the visited state list for this state as the list of parent + the current node
            visited_list = current_state.get_already_visited_nodes().copy()
            successor.set_already_visited_nodes(visited_list)
            # Set the parent node to the current node
            successor.set_parent_node(current_state)
            # Add to the successors list
            successors.append(successor)
    else:
        # We wont be completing the tour with the just next vertex
        visited = set(current_state.get_already_visited_nodes())
        for target in graph[current_state.get_node()].keys():
            # For every neighbour not visited yet
            if target not in visited:
                allowed_node_list = [node for node in graph.keys() if node not in visited and node != target]
                h_value = heuristic_function(graph,allowed_node_list,target,source)
                # No path will exist if mst does not exist
                if h_value is not False:
                    # Generate a new state
                    successor = state(target,None,None,None,None)
                    # Set the g-value of this state appropriately
                    successor.set_action_cost(current_state.get_action_cost() + graph[current_state.get_node()][target])
                    # Update the visited state list for this state as the list of parent + the current node
                    visited_list = current_state.get_already_visited_nodes().copy()
                    visited_list.append(target)
                    successor.set_already_visited_nodes(visited_list)
                    # Update the path order(Not required for final solution, just to display the progress of A*)
                    successor.set_path_order(current_state.get_path_order())
                    successor.append_path_order(target)
                    # Set the h-value of this state
                    successor.set_heuristic(h_value)
                    # Set the action from parent, used to create the final solution
                    successor.set_action(f'{current_state.get_node()}-{target}')
                    # Set the parent node to the current node
                    successor.set_parent_node(current_state)
                    # Add to the successors list
                    successors.append(successor)
    return successors

# Function to perform A* search for the TSP problem, using heap as a fringe list
def A_star_solver(graph, source):
    # Create the initial state, and get the initial heuristic estimate
    nodes = [key for key in graph.keys() if key != source]
    h_value = heuristic_function(graph,nodes,source,source)
    no_of_nodes_generated = 0
    no_of_nodes_expanded = 0

    if h_value:
        initial_state = state(source,0,h_value,[source],None) # Initial state created
        initial_state.set_parent_node(None)
        initial_state.set_path_order([source])
        no_of_nodes_generated += 1
        # Create the fringe list as a heap
        fringe_list = [initial_state]
        heapq.heapify(fringe_list)
        goal_state = None
        found = False
        while len(fringe_list) > 0 and not found:
            # Get the lowest f-cost node from the fringe list
            state_to_expand = heapq.heappop(fringe_list)
            # To display the progress of A*
            print(f'f-cost:{state_to_expand.get_action_cost()+state_to_expand.get_heuristic_cost()},path taken:{print_current_path(state_to_expand)}')
            # Make the goal test on this state
            if state_to_expand.get_heuristic_cost() == 0:
                # Goal node
                found = True
                goal_state = state_to_expand
            if not found:
                no_of_nodes_expanded += 1
                # Generate the successors of this state
                successors = Successor_function(state_to_expand,graph,source)
                for successor in successors:
                    heapq.heappush(fringe_list,successor)
                    no_of_nodes_generated += 1
        if found:
            # Construct the solution using parent nodes and action from parent
            rev_state = goal_state
            tsp_path = []
            while rev_state is not None:
                tsp_path.append(rev_state.get_action_from_parent())
                rev_state = rev_state.get_parent_node()
                
            # Print final tsp_path
            tsp_path.pop() # last element is None for initial state
            for step in reversed(tsp_path):
                print(step, end=" ")
            print("")
            print(f'Path cost found:{goal_state.get_action_cost()}')
            print(f'No.of nodes generated by A* : {no_of_nodes_generated}')
            print(f'No.of nodes expanded by A*: {no_of_nodes_expanded}')
        else:
            print('No solution')
    else:
        print('No solution')
    
if __name__ == '__main__':
    # Read filename 
    filename = input("Enter the name of the file:")
    try:
        graph = read_input(filename)
        nodes  = [n for n in graph.keys()]
        A_star_solver(graph,nodes[0])
    except Exception as e:
        print("Some error occured!")
        print(e)
