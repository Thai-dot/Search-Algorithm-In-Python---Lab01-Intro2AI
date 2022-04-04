import numpy as np
from sympy import false

# ------------------References------------------: 
# https://www.redblobgames.com/pathfinding/a-star/implementation.html
# https://python.plainenglish.io/uniform-cost-search-ucs-algorithm-in-python-ec3ee03fca9f
# https://stackoverflow.com/questions/8922060/how-to-trace-the-path-in-a-breadth-first-search
# https://www.youtube.com/watch?v=pcKY4hjDrxk&t=41s
# https://www.youtube.com/watch?v=dRMvK76xQJI&t=305s
# https://www.youtube.com/watch?v=6TsL96NAZCo
# https://www.youtube.com/watch?v=dv1m3L6QXWs&t=2s
# https://en.wikipedia.org/wiki/A*_search_algorithm

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
        #get the node with the lowest cost
        node = min(frontier.items(), key=lambda x: x[1][0])[0]
        #mark the node as visited
        visited.update({node: frontier[node][1]})

        #if the node is the end node, break the node
        if node == end:
            break
        #for each adjacent node
        for i in range(len(matrix[node])):
            #if the adjacent node is not visited
            if matrix[node][i] > 0 and i not in visited:
                #get the weight of the adjacent node plus the previous node cost
                new_cost = frontier.get(node)[0] + matrix[node][i]
                #check if the adjacent node is not in the frontier or if the new cost is lower than the old cost
                if i not in frontier or new_cost < frontier.get(i)[0]:
                    frontier.update({i: (new_cost, node)})

        del frontier[node]  # delete the node from the frontier

    # backtrack to get the path
    node = end
    while node != start:
        path.append(node)
        node = visited[node]
    path.append(start)
    # reverse the path
    path = path[::-1]

    return visited, path


def GBFS(matrix, start, end):

    # TODO:
    path = []
    visited = {}

    #create closedList and openList
    closedList = []
    openList = {}
    #push the first node into the openList
    openList.update({start: (0, None)})

    while openList:
        #get the node with the lowest cost
        node = min(openList.items(), key=lambda x: x[1][0])[0]
        #add the node to the closedList
        closedList.append(node)
        #mark the node as visited
        visited.update({node: openList[node][1]})
        #if the node is the end node, break the node
        if node == end:
            break
        #for each adjacent node
        for i in range(len(matrix[node])):
            #if the adjacent node is not visited
            if matrix[node][i] > 0 and i not in closedList:
                #get the weight of the adjacent node
                edge_weight = matrix[node][i]
                #check if the adjacent node is not in the openList or if the new cost is lower than the old cost
                if i not in openList or edge_weight < openList.get(i)[0]:
                    openList.update({i: (edge_weight, node)})

        del openList[node]

    #backtrack to get the path
    node = end
    while node != start:
        path.append(node)
        node = visited[node]
    path.append(start)
    #reverse the path
    path = path[::-1]

    return visited, path

#This function is a euclidean dist
#heuristic which is used to calculate the euclidean distance between two points
def euclidean_distance(x1, y1, x2, y2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def Astar(matrix, start, end, pos):
    # TODO:
    path = []
    visited = {}
    
    frontier = {}
    #Fn = g(n) + h(n), the first g(n) is zero, 
    #the second h(n) is the euclidean distance between the current node and the end node
    Fn = euclidean_distance(pos[end][0], pos[end]
                            [1], pos[start][0], pos[start][1])
    # push the first node into the frontier
    frontier.update({start: (0, None, Fn)})

    while frontier:
        #get the node with the lowest f(n)
        node = min(frontier.items(), key=lambda x: x[1][2])[0]
        #mark the node as visited
        visited.update({node: frontier[node][1]})
        #if the node is the end node, break the node
        if node == end:
            break
        #for each adjacent node
        for i in range(len(matrix[node])):
            #if the adjacent node is not visited
            if matrix[node][i] > 0 and i not in visited:
                #calculate the F(n) = g(n) + h(n) and new_cost
                new_Fn = frontier.get(node)[0] + matrix[node][i] + euclidean_distance(
                    pos[end][0], pos[end][1], pos[i][0], pos[i][1])
                new_cost = frontier.get(node)[0] + matrix[node][i]
                #if the adjacent node is not in the frontier or the new_Fn is smaller than the old Fn
                if i not in frontier or new_Fn < frontier.get(i)[2]:
                    frontier.update({i: (new_cost, node,new_Fn)}) #update the frontier

        del frontier[node]  # delete the node from the frontier after visit it

    #backtrack the path
    node = end
    while node != start:
        path.append(node)
        node = visited[node]
    path.append(start)
    #reverse the path
    path = path[::-1]
    return visited, path
