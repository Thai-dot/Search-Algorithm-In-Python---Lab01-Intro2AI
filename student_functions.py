import numpy as np
from queue import PriorityQueue

from sympy import false


def BFS(matrix, start, end):

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
    # TODO:
    path = []
    visited = {}

    frontier = {}
    # push the first node into the frontier
    frontier.update({start: (0, None)})

    while frontier:
        node = min(frontier.items(), key=lambda x: x[1][0])[0]
        visited.update({node: frontier[node][1]})

        if node == end:
            break

        for i in range(len(matrix[node])):
            if matrix[node][i] > 0 and i not in visited:
                new_cost = frontier.get(node)[0] + matrix[node][i]
                if i not in frontier or new_cost < frontier.get(i)[0]:
                    frontier.update({i: (new_cost, node)})

        del frontier[node]  # delete the node from the frontier

    node = end
    while node != start:
        path.append(node)
        node = visited[node]
    path.append(start)
    path = path[::-1]

    return visited, path


def GBFS(matrix, start, end):

    # TODO:
    path = []
    visited = {}

    closedList = []
    openList = {}
    openList.update({start: (0, None)})

    while openList:
        node = min(openList.items(), key=lambda x: x[1][0])[0]
        closedList.append(node)
        visited.update({node: openList[node][1]})
        if node == end:
            break
        for i in range(len(matrix[node])):
            if matrix[node][i] > 0 and i not in closedList:
                edge_weight = matrix[node][i]
                if i not in openList or edge_weight < openList.get(i)[0]:
                    openList.update({i: (edge_weight, node)})

        del openList[node]

    node = end
    while node != start:
        path.append(node)
        node = visited[node]
    path.append(start)
    path = path[::-1]

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
