import heapq

from grid import Node, NodeGrid
from math import inf
from heapq import heappush as pq_enqueue, heappop as pq_dequeue



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


    # Entre os algoritmos Dijkstra, Greedy e AStar, o que muda entre eles é o critério que eu uso para ordenar os elementos da fila de prioridades (respectivamente: g, h e f==g+h)
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
        path = []
        path_cost = inf

        if self.cost_map.is_occupied(start_position[0], start_position[1]):
            raise RuntimeError("Error! Starting vertex is occupied by an obstacle.")
        if self.cost_map.is_occupied(goal_position[0], goal_position[1]):
            raise RuntimeError("Error! Goal vertex is occupied by an obstacle.")

        # Initializing an empty priority queue
        # The elements of the priority queue are of the type "Node"
        pq = []
        # Attributing cost "0" to the starting vertex
        # self.cost_map.grid[start_position[0], start_position[1]] = \
        #     self.node_grid.grid[start_position[0], start_position[1]].g = 0.0
        self.cost_map.grid[start_position[0], start_position[1]] = 0.0
        self.node_grid.grid[start_position[0], start_position[1]].g = 0.0
        # Pushing the node "start_position" to the queue
        pq_enqueue(pq, self.node_grid.get_node(start_position[0], start_position[1]))

        # While the priority queue is not empty, proceed with the algorithm
        while len(pq) > 0:
            node_processing = pq_dequeue(pq)
            # 2-tuple of "int" containing the position of the node currently being processed
            position_processing = node_processing.get_position()
            # "float" containing the cost of the node currently being processed
            cost_node_pro = self.cost_map.get_cell_cost(position_processing[0], position_processing[1])
            # DEBUGGING
            print("cost_node_pro:", cost_node_pro)
            # Here, "successors" is a list of tuples, each tuple containing the position (2-tuple of int) of a successor of the node whose position is "start_position"
            successors = [self.node_grid.get_node(position[0], position[1]) \
                          for position in \
                          self.node_grid.get_successors(position_processing[0], position_processing[1])]

            for node_successor in successors:
                # Adding the successor node to the priority queue
                pq_enqueue(pq, node_successor)
                # 2-tuple of "int" containing the position of the successor node under analysis
                position_successor = node_successor.get_position()
                # "float" containing the cost of the successor node under analysis
                cost_node_suc = self.cost_map.get_cell_cost(position_successor[0], position_successor[1])
                # DEBUGGING
                print("cost_node_suc:", cost_node_suc)
                # "float" containing the cost of the edge going from the node being processed to its successor node under analysis
                cost_edge_pro2suc = self.cost_map.get_edge_cost(position_processing, position_successor)
                # DEBUGGING
                print("cost_edge_pro2suc:", cost_edge_pro2suc)

                # Checking if the successor node is occupied
                if cost_node_suc < 0.0:
                    raise TypeError("Error! A successor node cannot be occupied by an obstacle.")
                # Checking if the edge has a negative weight
                if cost_edge_pro2suc < 0.0:
                    raise ValueError("Error! An edge cannot have a negative weight.")



                if cost_node_pro + cost_edge_pro2suc < cost_node_suc:
                    # DEBUGGING:
                    print("Entered the most internal loop.")
                    # Reassigning the sucessor's parent to be the node being processed
                    self.node_grid.grid[position_successor[0], position_successor[1]].parent = \
                        node_processing
                    # Reassigning the successor node's cost
                    self.cost_map.grid[position_successor[0], position_successor[1]] =    \
                    self.node_grid.grid[position_successor[0], position_successor[1]].g = \
                        cost_node_pro + cost_edge_pro2suc


        # Making the heuristic function "float('nan')", because it is not used in Dijkstra
        for row in self.node_grid.grid:
            for node in row:
                node.f = node.g

        return path, path_cost

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
        path = []
        path_cost = inf

        return path, path_cost

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
        path = []
        path_cost = inf

        # DUVIDA: usar como heuristica a distancia euclidiana ?

        return path, path_cost
