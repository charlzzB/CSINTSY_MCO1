import heapq

class Graph:
    def __init__(self):
        # Initialize an empty dictionary to store edges
        self.edges = {}

    def add_edge(self, node1, node2, cost):
        # Add an edge from node1 to node2 with the given cost
        if node1 not in self.edges:
            self.edges[node1] = []
        self.edges[node1].append((node2, cost))

        # Add an edge from node2 to node1 with the given cost
        if node2 not in self.edges:
            self.edges[node2] = []
        self.edges[node2].append((node1, cost))

# Depth First Search function
def dfs(visited, map, node, goal, path):
    """
    Perform Depth-First Search (DFS) on the graph.

    @param visited: Set to keep track of visited nodes.
    @param map: Dictionary representing the adjacency list of the graph.
    @param node: The current node being visited.
    @param goal: The target node to reach.
    @param path: List to store the path from the start node to the goal node.
    @return: True if the goal node is found, False otherwise.
    """
    if node == goal:
        # If the current node is the goal, add it to the path and return True
        path.append(node)
        return True

    if node not in visited:
        # If the current node hasn't been visited, mark it as visited
        visited.add(node)
        # Add the current node to the path
        path.append(node)
        # Recur for all neighbors
        for neighbor in map[node]:
            if dfs(visited, map, neighbor, goal, path):
                return True
        # If the goal is not found, backtrack by removing the node from the path
        path.pop()

    return False

def costing(path, costings):
    """
    Calculate the total cost of the path.

    @param path: List of nodes representing the path.
    @param costings: Dictionary with edge costs.
    @return: Total cost of the path.
    """
    total_cost = 0
    # Sum the costs for each consecutive pair of nodes in the path
    for i in range(len(path) - 1):
        total_cost += costings.get((path[i], path[i + 1]), 0)
    return total_cost

def astar(graph, start, goal, heuristic_values):
    """
    Perform A* search on the graph.

    @param graph: Graph object containing the edges.
    @param start: The start node.
    @param goal: The goal node.
    @param heuristic_values: Dictionary containing heuristic values for each node.
    @return: Tuple containing the path and the total cost.
    """
    visited = set()
    # Initialize the priority queue with the start node, cost, and heuristic value
    queue = [(0 + heuristic(start, goal, heuristic_values), 0, start, [])]

    while queue:
        # Pop the node with the lowest total cost (current cost + heuristic)
        _, cost, node, path = heapq.heappop(queue)

        if node == goal:
            # If the goal node is reached, return the path and cost
            return path + [node], cost

        if node not in visited:
            # If the node hasn't been visited, mark it as visited
            visited.add(node)

            # Iterate over all neighbors of the current node
            for neighbor, neighbor_cost in graph.edges.get(node, []):
                if neighbor not in visited:
                    # Calculate the total cost for the neighbor
                    total_cost = cost + neighbor_cost + heuristic(neighbor, goal, heuristic_values)
                    # Calculate the new cost to reach the neighbor
                    new_cost = cost + neighbor_cost
                    # Push the neighbor to the priority queue
                    heapq.heappush(queue, (total_cost, new_cost, neighbor, path + [node]))

    return None, None

def get_heuristics(start, goal, graph):
    """
    Get heuristic values for A* search.

    @param start: The start node.
    @param goal: The goal node.
    @param graph: Graph object containing the edges.
    @return: Dictionary containing heuristic values for each node.
    """
    if start == "Dallas" and goal == "Chicago":
        # If the start and goal nodes are specific cities which are Dallas and Chicago, return pre-defined heuristic values
        return {
            'Miami': 2000,
            'New York': 800,
            'Boston': 900,
            'Dallas': 1200,
            'San Francisco': 2200,
            'Los Angeles': 2400,
            'Chicago': 0
        }
    else:
        # Otherwise, prompt the user to enter heuristic values for each node
        heuristics = {goal: 0}
        for node in graph.edges:
            if node != goal:
                heuristics[node] = int(input(f"Enter heuristic value for {node} (distance to {goal}): "))
        return heuristics

def heuristic(node, goal, heuristic_values):
    """
    Heuristic function for A* search.

    @param node: The current node.
    @param goal: The goal node.
    @param heuristic_values: Dictionary containing heuristic values for each node.
    @return: Heuristic value for the current node.
    """
    return heuristic_values.get(node, 0)

# Main program execution
# Define the cities and their connections
graph_cities = {
    'Dallas': ['Los Angeles', 'Miami', 'New York'],
    'Los Angeles': ['Dallas', 'New York', 'San Francisco'],
    'Miami': ['Dallas', 'New York'],
    'New York': ['Boston', 'Chicago', 'Dallas', 'Los Angeles', 'Miami'],
    'Boston': ['New York'],
    'San Francisco': ['Chicago', 'Los Angeles'],
    'Chicago': ['New York', 'San Francisco']
}

# Define the costs associated with each connection
graph_costs = {
    ('Dallas', 'Los Angeles'): 1700,
    ('Dallas', 'Miami'): 1200,
    ('Dallas', 'New York'): 1500,
    ('Los Angeles', 'Dallas'): 1700,
    ('Los Angeles', 'New York'): 3000,
    ('Los Angeles', 'San Francisco'): 500,
    ('New York', 'Boston'): 250,
    ('New York', 'Chicago'): 800,
    ('New York', 'Dallas'): 1500,
    ('New York', 'Los Angeles'): 3000,
    ('New York', 'Miami'): 1000,
    ('San Francisco', 'Chicago'): 2200,
    ('San Francisco', 'Los Angeles'): 500,
    ('Miami', 'Dallas'): 1200,
    ('Miami', 'New York'): 1000,
    ('Boston', 'New York'): 250,
    ('Chicago', 'New York'): 800,
    ('Chicago', 'San Francisco'): 2200
}

# List of all the visited cities
list_visited = set()
# Goal to reach (default)
goal_city = "Chicago"
# Start city (default)
start_city = "Dallas"
# List of all the cities to go through to get to the final goal
final_path = []

# Start
start_city = input("Which city do you want to START with?: ")
goal_city = input("Which city do you want to END with?: ")

# Initialize graph and heuristic values
graph = Graph()
# Add edges to the graph
for (city1, city2), cost in graph_costs.items():
    graph.add_edge(city1, city2, cost)

# Get heuristic values for the start and goal cities
heuristic_values = get_heuristics(start_city, goal_city, graph)

print("\nDepth-First Search:")
# Perform Depth-First Search (DFS)
if dfs(list_visited, graph_cities, start_city, goal_city, final_path):
    print("Final Path: ", " -> ".join(final_path))
    total_cost = costing(final_path, graph_costs)
    print("Total Cost: ", total_cost)
else:
    print("Path not found")

print("\nA* Search:")
# Perform A* Search
path, cost = astar(graph, start_city, goal_city, heuristic_values)
if path:
    print("Path: ", " -> ".join(path))
    print("Total cost:", cost)
else:
    print("No path found.")
