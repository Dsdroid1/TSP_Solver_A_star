"""
Contains the data structure used for storing the state information
"""
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
        # This var is only used to debug and show the progress of A*
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

    # Override comparison for min heap
    def __lt__(self,nxt):
        return self.action_cost+self.heuristic < nxt.action_cost+nxt.heuristic 

    # Print state info
    def print_state(self):
        print(f'Current Node:{self.node}')
        print(f'Cost to current path:{self.action_cost}')
        print(f'Heuristic to goal:{self.heuristic}')
        print(f'Action taken from parent:{self.action}')
        print(f'Nodes already visited in path:{self.already_visited_nodes}')