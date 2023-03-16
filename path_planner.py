from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Dijkstra algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        self.node_grid.reset()

        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])

        pq = [] # Creates the heap
        node = self.node_grid.get_node(start_position[0], start_position[1]) # Get start node
        node.f = 0 # Set first cost
        heapq.heappush(pq, (node.f, node)) # Insert the first node
        while len(pq) != 0:
            f, node = heapq.heappop(pq) # Extract priority node
            if not node.closed: # Verifies if node was already closed
                node.closed = True

                if node == goal_node: # If node was excracted of heap and is the goal node, return
                    return self.construct_path(goal_node), goal_node.f
                
                i_node, j_node = node.get_position() # Get position of priority node
                for successor_position in self.node_grid.get_successors(i_node, j_node): # For all positions of successor of priority node
                    successor = self.node_grid.get_node(successor_position[0], successor_position[1]) # Get successor node
                    if successor.f > node.f + self.cost_map.get_edge_cost((i_node, j_node), successor_position):
                        successor.f = node.f + self.cost_map.get_edge_cost((i_node, j_node), successor_position)
                        successor.parent = node
                        heapq.heappush(pq, (successor.f, successor))
        return [], inf

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Greedy Search algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        self.node_grid.reset()

        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])

        pq = [] # Creates the heap
        node = self.node_grid.get_node(start_position[0], start_position[1]) # Get start node
        node.f = node.distance_to(goal_position[0], goal_position[1]) # Set first cost
        node.g = 0
        heapq.heappush(pq, (node.f, node)) # Insert the first node
        while len(pq) != 0:
            f, node = heapq.heappop(pq) # Extract priority node
            if not node.closed: # Verifies if node was already closed
                node.closed = True

                i_node, j_node = node.get_position() # Get position of priority node
                for successor_position in self.node_grid.get_successors(i_node, j_node): # For all positions of successor of priority node
                    successor = self.node_grid.get_node(successor_position[0], successor_position[1]) # Get successor node
                    # Verifies if the cost of getting to the node is greater than the update.
                    # Working with graphs and not trees, avoid cycles.
                    if successor.g > node.g + self.cost_map.get_edge_cost((i_node, j_node), successor_position):
                        successor.parent = node
                        successor.g = node.g + self.cost_map.get_edge_cost((i_node, j_node), successor_position)
                        if successor == goal_node: # If node was excracted of heap and is the goal node, return
                            return self.construct_path(goal_node), goal_node.g
                        successor.f = successor.distance_to(goal_position[0], goal_position[1])
                        heapq.heappush(pq, (successor.f, successor))
        return [], inf

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the A* algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        self.node_grid.reset()

        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])

        pq = [] # Creates the heap
        node = self.node_grid.get_node(start_position[0], start_position[1]) # Get start node
        node.g = 0
        node.f = node.distance_to(goal_position[0], goal_position[1]) # Set first cost
        
        heapq.heappush(pq, (node.f, node)) # Insert the first node
        while len(pq) != 0:
            f, node = heapq.heappop(pq) # Extract priority node
            if not node.closed: # Verifies if node was already closed
                node.closed = True

                if node == goal_node: # If node was excracted of heap and is the goal node, return
                    return self.construct_path(goal_node), goal_node.g

                i_node, j_node = node.get_position() # Get position of priority node
                for successor_position in self.node_grid.get_successors(i_node, j_node): # For all positions of successor of priority node
                    successor = self.node_grid.get_node(successor_position[0], successor_position[1]) # Get successor node
                    # Verifies if the cost of getting to the node is greater than the update.
                    # Working with graphs and not trees, avoid cycles.
                    if successor.f > node.g + self.cost_map.get_edge_cost((i_node, j_node), successor_position) + successor.distance_to(goal_position[0], goal_position[1]):
                        successor.g = node.g + self.cost_map.get_edge_cost((i_node, j_node), successor_position)
                        successor.f = successor.g + successor.distance_to(goal_position[0], goal_position[1])
                        successor.parent = node
                        heapq.heappush(pq, (successor.f, successor))
        return [], inf