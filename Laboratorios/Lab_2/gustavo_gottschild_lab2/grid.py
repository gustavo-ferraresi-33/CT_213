import numpy as np
import random
from math import inf, sqrt

# DEBUGGING
import inspect


class CostMap(object):
    """
    Represents a cost map where higher values indicates terrain which are harder to transverse.
    """
    def __init__(self, width, height):
        """
        Creates a cost map.

        :param width: width (number of columns) of the cost map.
        :type width: int.
        :param height: height (number of rows) of the cost map.
        :type height: int.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        self.width = width
        self.height = height

        # BUG: the values were initialized as "1", but, now, they are initialized as "math.inf"
        # DUVIDA: "self.grid[i, j]" contem o custo para se sair do vertice de inicio e chegar ao vertice de posicao "(i, j)", certo?
        # DUVIDA: estah certo inicializar "self.grid[i, j]" com o valor 1.0
        self.grid = np.empty((height, width))
        for i in range(self.height):
            for j in range(self.width):
                if self.is_index_valid(i, j):
                    self.grid[i, j] = 1


    def get_cell_cost(self, i, j):
        """
        Obtains the cost of a cell in the cost map.

        :param i: the row (y coordinate) of the cell.
        :type i: int.
        :param j: the column (x coordinate) of the cell.
        :type j: int.
        :return: cost of the cell.
        :rtype: float.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        return self.grid[i, j]

    def get_edge_cost(self, start, end):
        """
        Obtains the cost of an edge.

        :param start: tbe cell where the edge starts.
        :type start: tuple of int.
        :param end: the cell where the edge ends.
        :type end: tuple of int.
        :return: cost of the edge.
        :rtype: float.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        if not abs(start[0] - end[0]) <= 1 or not abs(start[1] - end[1]) <= 1:
            raise ValueError("Error! Edge's start and end must have their coordinates differing by no more than 1 .")
        diagonal = (start[0] != end[0]) and (start[1] != end[1])
        factor = sqrt(2) if diagonal else 1.0
        # DUVIDA: por que o calculo do custo de cada aresta eh feito com a media dos custos das celulas de partida e de chegada? O custo nao deveria ser simplesmenteo o "factor" ?
        # RESPOSTA:
        # - Isso é uma carteacao
        # - A funcao dessa carteacao eh fazer com que os vertices proximos a obstaculos tenham um custo maior
        # - Isso estah certo, por incrivel que pareca
        return factor * (self.get_cell_cost(start[0], start[1]) + self.get_cell_cost(end[0], end[1])) / 2.0



    def is_occupied(self, i, j):
        """
        Checks if a cell is occupied.

        :param i: the row of the cell.
        :type i: int.
        :param j: the column of the cell.
        :type j: int.
        :return: True if the cell is occupied, False otherwise.
        :rtype: bool.
        """
        # DUVIDA: por que o criterio para saber se uma celula estar ocupada eh
        # return self.grid[i][j] < 0.0
        # RESPOSTA: porque, de acordo com a convencao do Manga
        #   -> custo valendo "1" significa "existente e livre"
        #   -> custo valendo "1" significa "existente e ocupado"

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        return self.grid[i, j] < 0.0

    def is_index_valid(self, i, j):
        """
        Check if a (i,j) position is valid (is within the map boundaries).

        :param i: the row of the cell.
        :param i: int.
        :param j: the column of the cell.
        :param j: int.
        :return: if the index is valid.
        :rtype: bool.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        return 0 <= i < self.height and 0 <= j < self.width

    def add_random_obstacle(self, width, height):
        """
        Adds a random obstacle to the map.

        :param width: width (number of columns) of the obstacle.
        :type width: int.
        :param height: height (number of rows) of the obstacle.
        :type height: int.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        top_left = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))
        self.add_obstacle((top_left[0], top_left[1], width, height))

    def add_obstacle(self, rectangle):
        """
        Adds an obstacle given a rectangular region (x, y, width, height).

        :param rectangle: a rectangle defined as (x, y, width, height), where (x, y) is the top left corner.
        :type rectangle: 4-dimensional tuple.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        # BUG:
        # DUVIDA: a implementacao do metodo "add_obstacle()" estah correta?
        self.add_rectangle((rectangle[0] - 1, rectangle[1] - 1, rectangle[2] + 2, rectangle[3] + 2), 2.0)
        self.add_rectangle(rectangle, -1.0)

    def add_rectangle(self, rectangle, value = -1.0):
        """
        Changes the values of a rectangular region to a given value.

        :param rectangle: rectangular region defined as (x, y, width, height), where (x, y) is the top left corner.
        :param value: the value used in the rectangular region.
        # DUVIDA: o que significa o parametro "value" ?
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        left = rectangle[0]
        right = rectangle[0] + rectangle[2]
        top = rectangle[1]
        bottom = rectangle[1] + rectangle[3]
        for j in range(left, right):
            for i in range(top, bottom):
                if self.is_index_valid(i, j) and not self.is_occupied(i, j):
                    self.grid[i, j] = value

    def create_random_map(self, obstacle_width, obstacle_height, num_obstacles):
        """
        Creates a random map by creating many random obstacles.

        :param obstacle_width: width (number of columns) of each obstacle.
        :type obstacle_width: int.
        :param obstacle_height: height (number of rows) of each obstacle.
        :type obstacle_height: int.
        :param num_obstacles: number of obstacles.
        :type num_obstacles: int.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        for i in range(num_obstacles):
            self.add_random_obstacle(obstacle_width, obstacle_height)


class NodeGrid(object):
    """
    Represents a grid of graph nodes used by the planning algorithms.
    """
    def __init__(self, cost_map):
        """
        Creates a grid of graph nodes.

        :param cost_map: cost map used for planning.
        # DUVIDA: o elemento "cost_map[i, j]"
        :type cost_map: CostMap.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        self.cost_map = cost_map
        self.width = cost_map.width
        self.height = cost_map.height
        self.grid = np.empty((self.height, self.width), dtype=Node)
        for i in range(np.size(self.grid, 0)):
            for j in range(np.size(self.grid, 1)):
                self.grid[i, j] = Node(i, j)


    def reset(self):
        """
        Resets all nodes of the grid.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        for row in self.grid:
            for node in row:
                node.reset()

    def get_node(self, i, j):
        """
        Obtains the node at row i and column j.

        :param i: row of the node.
        :type i: int.
        :param j: column of the node.
        :type j: int.
        :return: node at row i and column j.
        :rtype: Node.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        return self.grid[i, j]

    def get_successors(self, i, j):
        """
        Obtains a list of the 8-connected successors of the node at (i, j).

        :param i: row of the node.
        :type i: int.
        :param j: column of the node.
        :type j: int.
        :return: list of the 8-connected successors.
        :rtype: list of Node.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        successors = []
        for di in range(-1, 2):
            for dj in range(-1, 2):
                if di != 0 or dj != 0:
                    if self.cost_map.is_index_valid(i + di, j + dj) and not self.cost_map.is_occupied(i + di, j + dj):
                        successors.append((i + di, j + dj))
        return successors


class Node(object):
    """
    Represents a node of a graph used for planning paths.
    """
    def __init__(self, i=0, j=0):
        """
        Creates a node of a graph used for planning paths.

        :param i: row of the node in the occupancy grid.
        :type i: int.
        :param j: column of the node in the occupancy grid.
        :type j: int.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        self.i = i
        self.j = j

        # Initializing the total cost "f = g + h"
        self.f = inf
        # Initializing the node cost "g"
        self.g = inf
        # Initializing the heuristic cost "h"
        self.h = inf
        # DUVIDA: entao: "f = g + h", em que "g" é o custo do vertice e "h" é a heuristica do ate o noh objetivo

        # DUVIDA: o atributo "closed"eh uma booleana que vale "True" se o noh jah foi descoberto e visitado (isto eh: explorado) ?
        self.closed = False
        self.parent = None

        # Optimal-path finding algorithm to be used
        self.bpf = None

    def get_position(self):
        """
        Obtains the position of the node as a tuple.

        :return: (i, j) where i is the row and the column of the node, respectively.
        :rtype: 2-dimensional tuple of int.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        return self.i, self.j

    # DUVIDA: este metodo nao deveria ser "set_cost()"?
    def set_position(self, i, j):
        """
        Sets the position of this node.

        :param i: row of the node in the occupancy grid.
        :type i: int.
        :param j: column of the node in the occupancy grid.
        :type j: int.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        self.i = i
        self.j = j

    def reset(self):
        """
        Resets the node to prepare it for a new path planning.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        self.f = inf
        self.g = inf
        self.h = inf
        self.closed = False
        self.parent = None

    def distance_to(self, i, j):
        """
        Computes the distance from this node to the position (i, j).

        :param i: row of the target position.
        :type i: int.
        :param j: column of the target position.
        :type j: int.
        :return: distance from this node to (i, j).
        :rtype: float.
        """

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)

        return sqrt((self.i - i) ** 2 + (self.j - j) ** 2)


    # VERSAO ORIGINAL DO MANGA
    #
    # def __lt__(self, another_node):
    #
    #     # DEBUGGING:
    #     print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)
    #
    #     # DUVIDA: para que serve esta funcao? Nao vejo utilidade para ela que nao seja para construir um retangulo
    #     # RESPOSTA: significa "less than". Deixar como estah
    #     if self.i < another_node.i:
    #         return True
    #     if self.j < another_node.j:
    #         return True
    #     return False

    # MINHA VERSAO
    #
    def __lt__(self, another_node):

        # DEBUGGING:
        # print(type(self).__name__ + "." + inspect.currentframe().f_code.co_name)


        # Initial definitions
        nan = float("nan")

        # Checking which bpf will be used
        match self.bpf:
            case "dijkstra":
                return self.g < another_node.g
            case "greedy":
                return self.h < another_node.h
            case "a_star":
                return self.f < another_node.f
            case _:
                raise ValueError("Error! Illegal bpf algorithm.")

        # # Costs for the first node
        # f1 = self.f
        # g1 = self.g
        # h1 = self.h
        # # if not ((f1 == g1 == inf) or (f1 == nan) or (g1 == nan)):
        # #     h1 = self.f - self.g
        # # else:
        # #     h1
        #
        # # Costs for the second node
        # f2 = another_node.f
        # g2 = another_node.g
        # h2 = another_node.h
        # # if not ((f2 == g2 == inf) or (f2 == nan) or (g2 == nan)):
        # #     h2 = another_node.f - another_node.g
        # # else:
        # #     h2 = None
        #
        # if not ((inf in [f1, g1, h1, f2, g2, h2]) or (nan in [f1, g1, h1, f2, g2, h2])):
        #     if f1 < f2 or g1 < g2 or h1 < h2:
        #         return True
        #     if f1 > f2 or g1 > g2 or h1 > h2:
        #         return False
        #     elif f1 == f2 and g1 == g2 and h1 == h2:
        #         raise ValueError("Error! All costs (f, g and h) are equal.")
        #     else:
        #         raise ValueError("Error! None of the 3 conditions was entered.")
        # else:
        #
        #     print("f1:", f1)
        #     print("g1:", g1)
        #     print("h1:", h1)
        #     print("f2:", f2)
        #     print("g2:", g2)
        #     print("h2:", h2)
        #
        #     raise ValueError("Error! A node cost is still unitialized (inf).")

# Regras para fazer os labs direito:
# - Fazer com que o codigo que eu escrevo independa do resto do codigo
# - Nao criar e nem eliminar classes, metodos de classes, nem atributos