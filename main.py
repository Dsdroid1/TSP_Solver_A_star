import heapq

# Function to read the graph as an input file
def read_input(filename):
    """
    The format of the input file should be:
    (i) The first line should contain the number of nodes in the graph(will be numbered from 1 to n)
    (ii) The subsequent lines contain info about the edges of the graph, in the format:
        i j cost
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
            # print(file_extension)
            if file_extension == 'csv':
                separator = ','

        for line in lines:
            i,j,cost = list(map(int,line.strip().split(separator)))
            if i!=j:
                graph[i][j] = cost
                graph[j][i] = cost
        
        return graph 

# Function to calculate the cost of the Minimum Spanning Tree of the given graph, such that the MST only includes vertices from the 'allowed_node_list'
def MST(graph, allowed_node_list):
    # Currently using an N^2 implementation for testing, can be optimized further
    mst_costs = {}
    included_in_mst = {}
    final_heuristic = 0
    nodes_included = 0
    required_num_nodes = len(allowed_node_list) 
    # for key in graph.keys():
    #     if key in allowed_node_list:
    #         mst_costs[key] = None
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
                # print(key)
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
                # print(f'After 1 iteration we get,{min_cost_node} selected')
                # print(mst_costs)
            else:
                # MST does not exist
                return False
    return final_heuristic

def MST_with_heaps(graph, allowed_node_list):
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

def heuristic_function(graph, remaining_nodes, current_node, source):
    # The heuristic value is MST + path cost to closest node unvisited till now from the recent node + path cost of closest unvisited node from source
    nodes = [key for key in graph.keys()]
    # mst_cost = MST(graph,remaining_nodes)
    mst_cost = MST_with_heaps(graph,remaining_nodes)
    nodes_remaining = set(remaining_nodes)
    if mst_cost is not False:
        if len(remaining_nodes) != 0:
            current_node_to_closest_unvisited_node_cost = None
            closest_unvisited_node_to_source_vertex_cost = None
            for key in graph[current_node].keys():
                if key in nodes_remaining:
                    if current_node_to_closest_unvisited_node_cost == None or current_node_to_closest_unvisited_node_cost > graph[current_node][key]:
                        current_node_to_closest_unvisited_node_cost = graph[current_node][key]

            for key in graph[source].keys():
                if key in nodes_remaining:
                    if closest_unvisited_node_to_source_vertex_cost == None or closest_unvisited_node_to_source_vertex_cost > graph[source][key]:
                        closest_unvisited_node_to_source_vertex_cost = graph[source][key]
            
            # print(f'DEBUG-MST:{mst_cost}')
            # print(f'DEBUG-closest-node:{closest_unvisited_node_to_source_vertex_cost}')
            # print(f'DEBUG-current-to-closest')
            if closest_unvisited_node_to_source_vertex_cost != None and current_node_to_closest_unvisited_node_cost != None:
                return closest_unvisited_node_to_source_vertex_cost+mst_cost+current_node_to_closest_unvisited_node_cost
            else:
                return False     
        else:
            # No other nodes will be left, we need to pick the actual edge cost as the heuristic estimate
            edge_cost = graph[current_node].get(source)
            if edge_cost is not None:
                return edge_cost
            else:
                return False

    else:
        return False

# State data structure
class state:
    # Constructor function
    def __init__(self, node,action_cost,heuristic,already_visited_nodes,action):
        self.node = node
        self.action_cost = action_cost
        self.already_visited_nodes = already_visited_nodes
        self.heuristic = heuristic
        self.action = action
        self.parent = None
        self.path_order = []

    # Setters
    def set_node(self, node):
        self.node = node
    
    def set_action_cost(self, action_cost):
        self.action_cost = action_cost

    def set_action(self, action):
        self.action = action

    def set_heuristic(self, heuristic):
        self.heuristic = heuristic

    def set_already_visited_nodes(self, already_visited_nodes):
        self.already_visited_nodes = already_visited_nodes

    def set_parent_node(self, parent):
        self.parent = parent

    def set_path_order(self, path_order):
        self.path_order = path_order.copy()

    def append_path_order(self, new_node_in_path):
        self.path_order.append(new_node_in_path)

    # Getters
    def get_node(self):
        return self.node
    
    def get_action_cost(self):
        return self.action_cost

    def get_heuristic_cost(self):
        return self.heuristic

    def get_action_from_parent(self):
        return self.action

    def get_already_visited_nodes(self):
        return self.already_visited_nodes

    def get_parent_node(self):
        return self.parent
    
    def get_path_order(self):
        return self.path_order

    # Override comparison for heap
    def __lt__(self,nxt):
        return self.action_cost+self.heuristic < nxt.action_cost+nxt.heuristic 

    # Print state info
    def print_state(self):
        print(f'Current Node:{self.node}')
        print(f'Cost to current path:{self.action_cost}')
        print(f'Heuristic to goal:{self.heuristic}')
        print(f'Action taken from parent:{self.action}')
        print(f'Nodes already visited in path:{self.already_visited_nodes}')

def hash_string_of_path(state):
    path = state.get_path_order()
    hash_key = ""
    for node in path:
        hash_key += f"-{node}"
    return hash_key

def Successor_function(current_state, graph, source):
    successors = []
    # In successors, we store info about node, action cost, heuristic,action, already_visited_nodes
    if len(current_state.get_already_visited_nodes()) == len(graph.keys()):
        # All nodes visited once, means we now only need to connect to source vertex
        if graph[current_state.get_node()].get(source) is not None:
            target = source
            successor = state(target,None,None,None,None)
            # successor['node'] = target
            # successor['action_cost'] = current_state['action_cost'] + graph[current_state['node']][target]
            successor.set_action_cost(current_state.get_action_cost() + graph[current_state.get_node()][target])
            # successor['heuristic'] = 0 # Goal state
            successor.set_heuristic(0)
            #########DEBUG
            successor.set_path_order(current_state.get_path_order())
            successor.append_path_order(target)
            # successor['action'] = f'{current_state["node"]}-{source}'
            successor.set_action(f'{current_state.get_node()}-{source}')
            visited_list = current_state.get_already_visited_nodes().copy()
            successor.set_already_visited_nodes(visited_list)
            successor.set_parent_node(current_state)
            successors.append(successor)
    else:
        visited = set(current_state.get_already_visited_nodes())
        for target in graph[current_state.get_node()].keys():
            
            if target not in visited:
                # allowed_node_list = []
                # for node in graph.keys():
                #     if node not in current_state.get_already_visited_nodes() and node != target:
                #         allowed_node_list.append(node)
                allowed_node_list = [node for node in graph.keys() if node not in visited and node != target]
                # print(allowed_node_list)
                # h_value = MST(graph,allowed_node_list)
                h_value = heuristic_function(graph,allowed_node_list,target,source)
                # No path will exist if mst does not exist
                if h_value is not False:
                    successor = state(target,None,None,None,None)
                    # successor['node'] = target
                    # successor['action_cost'] = current_state['action_cost'] + graph[current_state['node']][target]
                    successor.set_action_cost(current_state.get_action_cost() + graph[current_state.get_node()][target])
                    
                    
                    visited_list = current_state.get_already_visited_nodes().copy()
                    visited_list.append(target)
                    successor.set_already_visited_nodes(visited_list)
                    # successor['heuristic'] = MST(graph,allowed_node_list)
                    ######DEBUG
                    successor.set_path_order(current_state.get_path_order())
                    successor.append_path_order(target)
                    successor.set_heuristic(h_value)
                    # successor['action'] = f'{current_state["node"]}-{target}'
                    successor.set_action(f'{current_state.get_node()}-{target}')
                    successor.set_parent_node(current_state)
                    successors.append(successor)
    return successors



# Function to perform A* search for the TSP problem, using heaps to as fringe lists
def A_star_solver(graph, source):
    # Create the initial state
    nodes = [key for key in graph.keys() if key != source]
    h_value = heuristic_function(graph,nodes,source,source)
    no_of_nodes_generated = 0
    no_of_nodes_expanded = 0
    # closed_list = {}
    if h_value:
        # print(h_value)
        initial_state = state(source,0,h_value,[source],None)
        initial_state.set_parent_node(None)
        initial_state.set_path_order([source])
        no_of_nodes_generated += 1
        # Create the fringe list as a heap
        fringe_list = [initial_state]
        heapq.heapify(fringe_list)
        goal_state = None
        # Use heapq.heappush(fringe_list,element)
        found = False
        while len(fringe_list) > 0 and not found:
            # print('NEW ITER')
            state_to_expand = heapq.heappop(fringe_list)
            # print(state_to_expand.get_path_order())
            print(state_to_expand.get_action_cost()+state_to_expand.get_heuristic_cost(),hash_string_of_path(state_to_expand))
            # key = hash_string_of_path(state_to_expand)
            # if closed_list.get(key) is None:
                # Insert this node into the closed list
                # closed_list[key] = 1
                # print(key)
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
            # goal_state.print_state()
            rev_state = goal_state
            tsp_path = []
            while rev_state is not None:
                # rev_state.print_state()
                tsp_path.append(rev_state.get_action_from_parent())
                # print(" ")
                rev_state = rev_state.get_parent_node()
                
            # Print final tsp_path
            tsp_path.pop() # last element is None for initial state
            # print(tsp_path)
            for step in reversed(tsp_path):
                print(step, end=" ")
            print(f'Path cost found:{goal_state.get_action_cost()}')
            print(f'No.of nodes generated by A* : {no_of_nodes_generated}')
            print(f'No.of nodes expanded by A*: {no_of_nodes_expanded}')
        else:
            print('No solution')
    else:
        print('No solution')
    
if __name__ == '__main__':
    # graph = read_input('graph.txt')
    # Read filename 
    filename = input("Enter the name of the file:")
    try:
        graph = read_input(filename)
        nodes  = [n for n in graph.keys()]
        # print(graph)
        # print(nodes)
        # print(nodes)
        # retval = MST(graph,nodes)
        # print(retval)
        # current_state =state(1,0,MST(graph,nodes),[1],None)
        # current_state = state(3,16,15,[1,5,4,3],None)
        # retval = Successor_function(current_state,graph,1)
        # for cstate in retval:
        #     cstate.print_state()
        #     print(" ")
        #     # print(f'{cstate.get_node()}-{cstate.get_already_visited_nodes()}-{cstate.get_heuristic_cost()}-{cstate.get_action_cost()}')
        # print(MST(graph,nodes))
        A_star_solver(graph,1)
    except Exception as e:
        print("Some error occured!")
        print(e)
