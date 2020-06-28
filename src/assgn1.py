from heapq import *
from copy import deepcopy


def read_maze(filename):
    """
    -------------------------------------------------------
    creates a 2d list of the maze in a textfile given
    Use: readmaze("file.txt")
    -------------------------------------------------------
    Parameters:
        filename - textfile of maze (str)
    Returns:
         maze - maze % - walls, P - starting point, . - endpoint (2Dlist)   
    ------------------------------------------------------
    """
    fp = open(filename, "r")
    maze = []
    for line in fp:
        maze.append(list(line.strip()))
    fp.close()
    return maze


def find_start(maze):
    """
    -------------------------------------------------------
    Finds Pac man and he exit
    Use: starting_point= find_start(maze)
    -------------------------------------------------------
    Parameters:
        maze -  (2D list)
    Returns:
        starting_point - x,y coordinates (list)
    ------------------------------------------------------
    """
    start_point, end_point = None, None
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] == 'P': 
                start_point = [i, j]
            elif maze[i][j] == '.': 
                end_point = [i, j]
            found = start_point and end_point
            if found:
                break  
            
    return start_point , end_point 


class Node:

    def __init__(self, list, end, prev):
        """
        -------------------------------------------------------
        Point on a map
        Use: node = Node(list, end, prev)
        -------------------------------------------------------
        Parameters:
            list - x,y coordinates  (list)
            end - x,y coordinates of end (list)
            prev - previous (Node)
        Returns:
            node - a node of a point (Node)
        ------------------------------------------------------
        """
        self.list = list
        self.cost = self.h(end)
        self.prev = prev
        self.end = end
        
    def __eq__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other is self (Node)
        ------------------------------------------------------
        """
        return self.list == other.list
     
    def __ne__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other cost is not self (Node)
        ------------------------------------------------------
        """
        return self.list != other.list
     
    def __lt__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other cost is less than self (Node)
        ------------------------------------------------------
        """
        return self.cost < other.cost
     
    def __le__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other is less than or equal to self (Node)
        ------------------------------------------------------
        """
        return self.cost <= other.cost
     
    def __gt__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other is greater than self (Node)
        ------------------------------------------------------
        """
        return self.cost > other.cost
     
    def __ge__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other is greater than or equal to  self (Node)
        ------------------------------------------------------
        """
        return self.cost >= other.cost
    
    def h(self, end):
        """
        -------------------------------------------------------
        Manhattan distance between the endpoint and self
        Use: dist = h(end)
        -------------------------------------------------------
        Parameters:
            end - xy coordinate of endpoint (list)
        Returns:
            dist -  dist between nodes (int)
        ------------------------------------------------------
        """
#         node = self.list
#         x,y = end[0] - node[0], end[1] - node[1]
#         return sqrt(x**2 + y**2)
        distance = abs(end[0] - self.list[0]) + abs(end[1] - self.list[1])
        return distance
    
    def goal_test(self):
        """
        -------------------------------------------------------
        Checks if node is the goal
        Use: node.goal_test()
        -------------------------------------------------------
        Parameters:
            
        Returns:
            True or False(boolean)
        ------------------------------------------------------
        """
        return self.list == self.end
    
    def action(self, maze):
        """
        -------------------------------------------------------
        Finds all possible actions i.e. whether we could move up,
        down, left and right
        Use: actions = node.action(maze)
        -------------------------------------------------------
        Parameters:
            maze -> maze we are solving (2Dlist)
        Returns:
            actions -> a list of coordinates list(4x2) - list(1x2)
        ------------------------------------------------------
        """
        neighbours = []
        coord = self.list
        if coord[0] < len(maze) - 1 and maze[coord[0] - 1][coord[1]] != '%':
            neighbours.append([coord[0] - 1, coord[1]])
        if coord[0] > 0 and maze[coord[0] + 1][coord[1]] != '%': 
            neighbours.append([coord[0] + 1, coord[1]])
        if coord[1] > 0  and maze[coord[0]][coord[1] + 1] != '%':
            neighbours.append([coord[0], coord[1] + 1])
        if coord[1] < len(maze[0]) - 1 and maze[coord[0]][coord[1] - 1] != '%' :
            neighbours.append([coord[0], coord[1] - 1])
        return neighbours
    
    def solution(self, maze, expanded):
        """
        -------------------------------------------------------
        Draws path on the maze nad finds the cost to solve the maze
        Use: maze, path_cost, expanded, path = node.solution(maze, exapnded)
        -------------------------------------------------------
        Parameters:
            maze -> maze we are solving (2Dlist)
            expanded -> number of nodes expanded by search (int)
        Returns:
            maze -> solved maze (2D list)
            path_cost -> cost of solving maze (int)
            expanded -> number of nodes expanded by search (int)
            path -> a list of the path (list)
        ------------------------------------------------------
        """
        path_cost = 0
        node = self
        path = []
        while node.prev is not None:
            coord = node.list
            path_cost += 1 
            path.append(coord)
            maze[coord[0]][coord[1]] = '.'
            node = node.prev
        return maze, path_cost, expanded, path

    
class Node2:

    def __init__(self, list1, end, prev, start):
        """
        -------------------------------------------------------
        Point on a map, Keeps track of cost to get to this point
        Use: node = Node(list, end, prev, start)
        -------------------------------------------------------
        Parameters:
            list - x,y coordinates  (list)
            end - x,y coordinates of end (list)
            prev - previous (Node)
            start - x,y coordinate of starting point (list)
        Returns:
            node - a node of a point (Node)
        ------------------------------------------------------
        """
        self.list = list1
        self.prev = prev
        self.end = end
        self.path_cost = 0
        if self.prev is not None:
            self.path_cost = prev.path_cost + 1 
        self.cost = self.f(end)

    def __eq__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other is self (Node)
        ------------------------------------------------------
        """
        return self.list == other.list and self.list[1] == other.list[1]
     
    def __ne__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other cost is not self (Node)
        ------------------------------------------------------
        """
        return self.list[0] != other.list[0] or self.list[1] != other.list[1]
     
    def __lt__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other cost is less than self (Node)
        ------------------------------------------------------
        """
        return self.cost < other.cost
     
    def __le__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other is less than or equal to self (Node)
        ------------------------------------------------------
        """
        return self.cost <= other.cost
     
    def __gt__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other is greater than self (Node)
        ------------------------------------------------------
        """
        return self.cost > other.cost
     
    def __ge__(self, other):
        """
        -------------------------------------------------------
        Compares Nodes
        Use: 
        -------------------------------------------------------
        Parameters:
            other - node to compare to self (Node)
        Returns:
             true or false - if other is greater than or equal to  self (Node)
        ------------------------------------------------------
        """
        return self.cost >= other.cost
    
    def h(self, end):
        """
        -------------------------------------------------------
        shortest distance between the endpoint and self
        Use: dist = h(end)
        -------------------------------------------------------
        Parameters:
            end - xy coordinate of endpoint (list)
        Returns:
            dist -  dist between nodes (double)
        ------------------------------------------------------
        """
#         node = self.list
#         x,y = end[0] - node[0], end[1] - node[1]
#         return sqrt(x**2 + y**2)
        distance = abs(self.list[0] - end[0]) + abs(self.list[1] - end[1])
        return distance
    
    def f(self, end):
        """
        -------------------------------------------------------
        Path cost to get to this point plus distance 
        between node and end point
        Use: dist = f(end, start)
        -------------------------------------------------------
        Parameters:
            end - xy coordinate of endpoint (list)
        Returns:
            dist -  dist between nodes (double)
        ------------------------------------------------------
        """
        distance = self.h(end) + self.path_cost
        return distance
    
    def goal_test(self):
        """
        -------------------------------------------------------
        Checks if node is the goal
        Use: node.goal_test()
        -------------------------------------------------------
        Parameters:
            
        Returns:
            True or False(boolean)
        ------------------------------------------------------
        """
        return self.list == self.end
    
    def action(self, maze):
        """
        -------------------------------------------------------
        Finds all possible actions i.e. whether we could move up,
        down, left and right
        Use: actions = node.action(maze)
        -------------------------------------------------------
        Parameters:
            maze -> maze we are solving (2Dlist)
        Returns:
            actions -> a list of coordinates list(4x2) - list(1x2)
        ------------------------------------------------------
        """
        neighbours = []
        coord = self.list
        if coord[0] < len(maze) - 1 and maze[coord[0] - 1][coord[1]] != '%':
            neighbours.append([coord[0] - 1, coord[1]])
        if coord[0] > 0 and maze[coord[0] + 1][coord[1]] != '%': 
            neighbours.append([coord[0] + 1, coord[1]])
        if coord[1] > 0  and maze[coord[0]][coord[1] + 1] != '%':
            neighbours.append([coord[0], coord[1] + 1])
        if coord[1] < len(maze[0]) - 1 and maze[coord[0]][coord[1] - 1] != '%' :
            neighbours.append([coord[0], coord[1] - 1])
        return neighbours
    
    def solution(self, maze, expanded):
        """
        -------------------------------------------------------
        Draws path on the maze nad finds the cost to solve the maze
        Use: maze, path_cost, expanded = node.solution(maze, exapnded)
        -------------------------------------------------------
        Parameters:
            maze -> maze we are solving (2Dlist)
            expanded -> number of nodes expanded by search (int)
        Returns:
            maze -> solved maze (2D list)
            path_cost -> cost of solving maze (int)
            expanded -> number of nodes expanded by search (int)
            path -> path to exit from start (ordered list) 
        ------------------------------------------------------
        """
        node = self
        path_cost = 0
        path = []
        while node.prev is not None:
            coord = node.list
            path.append(coord)
            maze[coord[0]][coord[1]] = '.'
            path_cost += 1
            node = node.prev
        return maze, path_cost, expanded, path             


def bfs(maze, initial_point, endpoint):
    """
    -------------------------------------------------------
    breadth first search 
    Use: maze = bfs(maze, initial_point)
    -------------------------------------------------------
    Parameters:
        maze -  (2D list)
        initial_point - starting point x,y coordinates (list)
        endpoint - end point x,y coordinates (list)
    Returns:
        solution - the solved maze (Node function)
    ------------------------------------------------------
    """
    expanded = 1
    node = Node(initial_point, endpoint, None)
    if node.goal_test(): return node.solution(maze, expanded)
    stack = []
    stack.append(node)
    visited = []
    
    while len(stack) > 0:
        node = stack.pop(0)
        visited.append(node)
        for i in node.action(maze):
            child = Node(i, endpoint, node)
            if child not in visited and child not in stack:
                if child.goal_test(): return child.solution(maze, expanded)
                stack.append(child); expanded += 1
    return Node(initial_point, endpoint, None).solution(maze, 0)

    
def dfs(maze, initial_point, endpoint):
    """
    -------------------------------------------------------
    breadth first search 
    Use: maze = bfs(maze, initial_point)
    -------------------------------------------------------
    Parameters:
        maze -  (2D list)
        initial_point - starting point x,y coordinates (list)
        endpoint - end point x,y coordinates (list)
    Returns:
        solution - the solved maze (Node function)
    ------------------------------------------------------
    """
    expanded = 1
    node = Node(initial_point, endpoint, None)
    if node.goal_test(): return node.solution(maze, expanded)
    q = []
    q.append(node)
    visited = []
    
    while len(q) > 0:
        node = q.pop()
        visited.append(node)
        for i in node.action(maze):
            child = Node(i, endpoint, node)
            if child not in visited and child not in q:
                if child.goal_test(): return child.solution(maze, expanded)
                q.append(child); expanded += 1
    return Node(initial_point, endpoint, None).solution(maze, 0)


def gbs(maze, starting_point, endpoint):
    """
    -------------------------------------------------------
    greedy best search 
    Use: maze = gbs(maze, initial_point)
    -------------------------------------------------------
    Parameters:
        maze -  (2D list)
        starting_point - starting point x,y coordinates (list)
        endpoint - end point x,y coordinates (list)
    Returns:
        solution - the solved maze (Node function)
    ------------------------------------------------------
    """
    expanded = 1
    node = Node(starting_point, endpoint, None)
    heap = []
    visited = []
    heappush(heap, node)
    
    while len(heap) > 0:
        state = heappop(heap)
        visited.append(state)
        if state.goal_test(): return state.solution(maze, expanded)
        for i in state.action(maze):
            child = Node(i, endpoint, state)
            if child not in visited and child not in heap:
                heappush(heap, child); expanded += 1
            elif child in heap:
                l = heap.index(child)
                if child.cost < heap[l].cost :
                    heap[l] = child

    return Node(starting_point, endpoint, None).solution(maze, 0) 


def a_star(maze, starting_point, endpoint):
    """
    -------------------------------------------------------
    A* search 
    Use: maze = a_star(maze, initial_point)
    -------------------------------------------------------
    Parameters:
        maze -  (2D list)
        starting_point - starting point x,y coordinates (list)
        endpoint - end point x,y coordinates (list)
    Returns:
        solution - the solved maze (Node function)
    ------------------------------------------------------
    """
    expanded = 1
    node = Node2(starting_point, endpoint, None, starting_point)
    heap = []
    visited = []
    heappush(heap, node)
    
    while len(heap) != 0:
        node = heappop(heap)
        if node.goal_test(): return node.solution(maze, expanded)
        visited.append(node)
        for i in node.action(maze):
            child = Node2(i, endpoint, node, starting_point)
            if child not in heap and child not in visited:
                heappush(heap, child)
                expanded += 1
            elif child in heap:
                l = heap.index(child)
                if child.cost < heap[l].cost :
                    heap[l] = child
    
    return Node2(starting_point, endpoint, None, starting_point).solution(maze, expanded)


def for_gui(maze, alg):
    """
    -------------------------------------------------------
    Invoked by the GUI.py provides the path, cost generated by algorithm
    and how many nodes were expanded
    Use: path, count, expanded, start = for_gui(maze, alg)
    -------------------------------------------------------
    Parameters:
        maze - name of maze file (str)
        alg - which algorithm to implement (str)
    Returns:
        path - list of path to exit (list)
        count - the cost of the path (int)
        expanded - number of nodes visited (int)
        start - the staring coordinate ([x,y])
    ------------------------------------------------------
    """
    maze = read_maze(maze)
    starting_point, endpoint = find_start(maze)
    if alg == "dfs" : alg = dfs
    elif alg == "bfs" : alg = bfs
    elif alg == "gbs" : alg = gbs
    else:alg = a_star
    maze, count, expanded1, path = alg(deepcopy(maze), starting_point, endpoint)
    return path[::-1], count, expanded1, starting_point

# def solve(maze):
#     starting_point, endpoint= find_start(maze)
#     print(abs(starting_point[0] -endpoint[0]  ) + abs( starting_point[1] - endpoint[1]))
#     a_star_maze, n, expanded1, path = a_star(deepcopy(maze), starting_point, endpoint)
#     gbs_maze, m, expanded2, path = gbs(deepcopy(maze), starting_point, endpoint)
#     dfs_maze, l, expanded3, path = dfs(deepcopy(maze), starting_point, endpoint)   
#     bfs_maze, k, expanded4, path = bfs(deepcopy(maze), starting_point, endpoint)
#     print("\n\nA*:\nCount = {}\nExpanded = {} \nPath:".format(n, expanded1))
#     for i in a_star_maze: print("".join(i))
#     print("\n\nGbs:\nCount = {}\nExpanded = {} \nPath:".format(m, expanded2))
#     for i in gbs_maze: print("".join(i))
#     print("\n\nDfs: \nCount = {}\nExpanded = {} \nPath:".format(l, expanded3))
#     for i in dfs_maze: print("".join(i))   
#     print("\n\nBfs:\nCount = {}\nExpanded = {} \nPath:".format(k, expanded4))
#     for i in bfs_maze: print("".join(i))
# 
# maze = "mediumMaze.txt"
# maze1 = "large_maze.txt"
# for i in  [maze, maze1]:
#     maze = read_maze(i)
#     print(i + '\n')
#     solve(maze)
