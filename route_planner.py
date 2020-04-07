import math
from queue import PriorityQueue

def shortest_path(M, start, goal):
    """Find the shortest path from input: start to input: goal

    Keyword arguments:
    M -- the map for the connect network
    start -- the input starting point, referring to an intersection point on the map (M)
    goal -- the input ending point, referring to an intersection point on the map (M)
    
    Return route_finder function
    """
    # using Python's priority queue
    q = PriorityQueue()
    # [0] for h value / [1] for node 
    q.put((0, start)) 
    # updating which node is the previous node
    prev_node = {} 
    # keeping track of previous g values
    prev_g = {start: 0} 
    
    while not q.empty():
        # pop the node with the lowest h value
        current = q.get() 
        # return route if node is the goal
        if current[1] == goal:
            return route_finder(prev_node, start, goal)
        # loop through each child
        for child in M.roads[current[1]]:
            # find the g costs
            prev_g_cost = prev_g[current[1]] 
            path_cost = calculate_path_cost(M, current[1], child)
            total_g_cost = prev_g_cost + path_cost
            # h cost
            estimated_distance = calculate_h(M, child, goal)
            # f cost (total cost)
            total_cost = total_g_cost + estimated_distance
            # if child has been calculated before, and check if previous g value is still the lowest
            if child in prev_g and prev_g[child] < total_g_cost:
                # previous g value is lower, so skip the current child (don't add the current child to the priority queue)
                continue
            # g value can be lower
            prev_g[child] = total_g_cost
            # update the new previous
            prev_node[child] = current[1]
            # add to priority queue
            q.put((total_cost, child))


def calculate_path_cost(M, current, next_node):
    """Calculate the path cost between two points / nodes by using Pythagoras

    Keyword arguments:
    M -- the map for the connect network
    current -- point A
    next_node -- point B
    
    Returns the path cost (float)
    """
    # shortening 
    M_i = M.intersections
    # preparing for Pythagoras
    a = abs(M_i[next_node][0] - M_i[current][0])
    b = abs(M_i[next_node][1] - M_i[current][1])
    # calculate the hypotenuse from current to next child
    path_cost = math.sqrt(a ** 2 + b ** 2)
    return path_cost


def calculate_h(M, child, goal):
    """Calculate the heuristic value between two points / nodes by using Pythagoras

    Keyword arguments:
    M -- the map for the connect network
    child -- point A
    goal -- point B
    
    Returns the heuristic value (float)
    """
    # shortening 
    M_i = M.intersections
    # preparing for Pythagoras
    a = abs(M_i[goal][0] - M_i[child][0])
    b = abs(M_i[goal][1] - M_i[child][1])
    # calculate the hypotenuse from child to goal
    h_distance = math.sqrt(a ** 2 + b ** 2)
    return h_distance 


def route_finder(prev_dict, start, goal):
    """Create an array of integers to represent the sequence of intersection visits using the given dictionary

    Keyword arguments:
    prev_dict -- a dictionary mapping the previous node for a given node
    start -- starting point
    goal -- ending point
    
    Returns the path (list)
    """
    # initialise
    path = [goal]
    next_node = goal
    # iterate through the dictionary
    while next_node != start:
        next_node = prev_dict[next_node]
        path.append(next_node)
    # reverse the list and return
    path.reverse()
    return path
        