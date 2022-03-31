import numpy as np


def BFS(matrix, start, end):
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node

    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO:

    path = []
    visited = {}

    queue = []  # queue for BFS
    # push the first node into the queue
    queue.append([start])
    # mark the first node as visited
    visited.update({start: None})

    while queue:

        path = queue.pop(0)  # pop the first node from the queue
        node = path[-1]  # get the last node in the path

        if node == end:  # if the last node is the end node, break the node
            break
        for i in range(len(matrix[node])):
            if matrix[node][i] > 0 and i not in visited:  # if the adjacent node is not visited
                new_path = list(path)  # copy the old path
                new_path.append(i)  # append the adjacent node to the path
                queue.append(new_path)  # append the path to the queue
                visited.update({i: node})  # mark the adjacent node as visited

    return visited, path


def DFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node

    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO:

    path = []
    visited = {}

    stack = []  # stack for DFS
    # push the first node into the stack
    stack.append([start])
    # mark the first node as visited
    visited.update({start: None})

    while stack:

        path = stack.pop()  # pop the last node from the stack
        node = path[-1]  # get the last node in the path

        if node == end:  # if the last node is the end node, break the node
            break
        for i in range(len(matrix[node])):
            if matrix[node][i] > 0 and i not in visited:
                new_path = list(path)  # copy the old path
                new_path.append(i)  # append the adjacent node to the path
                stack.append(new_path)  # append the path to the stack
                visited.update({i: node})  # mark the adjacent node as visited

    return visited, path


def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node

    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:
    path = []
    visited = {}

    frontier = {}  # frontier for UCS
    # push the first node into the frontier
    frontier.update({start: 0})
    visited.update({start: None})

    while frontier:
        node = min(frontier, key=frontier.get) # get the node with the smallest cost
        path.append(node)  # path for UCS

        if node == end:  # if the last node is the end node, break the node
            break
        for i in range(len(matrix[node])):
            if matrix[node][i] > 0 and i not in visited:
                new_cost = frontier.get(node) + matrix[node][i]
                if i not in frontier or new_cost < frontier.get(i):
                    frontier.update({i: new_cost})
                    visited.update({i: node})
        del frontier[node]  # delete the node from the frontier
    print(path)
    print(visited)
    return visited, path


def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node

    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:
    path = []
    visited = {}
    return visited, path


def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:
    path = []
    visited = {}
    return visited, path
