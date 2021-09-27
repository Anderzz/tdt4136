import heapq
import numpy as np
from warnings import warn
from Map import *

class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
      return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    #needed for the priority queue
    def __lt__(self, other):
      return self.f < other.f
    
    def __gt__(self, other):
      return self.f > other.f

def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  #return the reversed path

def heruistic(actual_pos, goal_pos, norm="manhattan"):
  if norm=="manhattan" or "L1":
    return np.abs(actual_pos[0]-goal_pos[0]) + np.abs(actual_pos[1]-goal_pos[1])
  if norm=="euclidean" or "L2":
    return np.sqrt(np.abs(actual_pos[0]-goal_pos[0])**2 + np.abs(actual_pos[1]-goal_pos[1])**2)


def astar(map, start, end, allow_diagonal_movement = False):
    """
    Returns the path as a list of tuples from start to end in the given map
    :param map:
    :param start:
    :param end:
    :return:
    """

    #setup the start and goalnodes
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    goal_node = Node(None, end)
    goal_node.g = goal_node.h = goal_node.f = 0

    #initialize the open and closed lists
    open_list = []
    closed_list = []

    #heapify the open_list and add the start node
    heapq.heapify(open_list) 
    heapq.heappush(open_list, start_node)

    #add a stopping condition
    outer_iterations = 0
    #max_iterations = (len(map[0]) * len(map) // 2)
    max_iterations=100000

    #define our action space
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    #loop until you find the goal node
    while len(open_list) > 0:
        outer_iterations += 1

        #if outer_iterations > max_iterations:
          #return the current path if we hit
          #our stopping condition
        #  warn("giving up on pathfinding too many iterations")
        #  return return_path(current_node)       
        
        #get the current node
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        #if we find the goal
        if current_node == goal_node:
            return return_path(current_node)

        #generate the children
        children = []
        
        for new_position in adjacent_squares:

            #get the node positions
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            #make sure they are within range
            if node_position[0] > (len(map) - 1) or node_position[0] < 0 or node_position[1] > (len(map[len(map)-1]) -1) or node_position[1] < 0:
                continue

            #avoid roadblocks
            if map[node_position[0]][node_position[1]] != 0:
                continue

            #create a new node and append it
            new_node = Node(current_node, node_position)
            children.append(new_node)

        #for every child
        for child in children:
            #check if they are present on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            #make the cost function
            child.g = current_node.g + 1
            #child.h = ((child.position[0] - goal_node.position[0]) ** 2) + ((child.position[1] - goal_node.position[1]) ** 2)
            child.h=heruistic(child.position,goal_node.position,"L1")
            child.f = child.g + child.h

            #if the child is already on the open list
            if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            #add the child to the open list
            heapq.heappush(open_list, child)
    warn("Couldn't get a path to destination")
    return None

def example(print_map = True):

    map = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,] * 2,
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,] * 2,
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,] * 2,
            [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,] * 2,
            [0,0,0,1,1,0,0,1,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,] * 2,
            [0,0,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,] * 2,
            [0,0,0,1,0,1,1,1,1,0,1,1,0,0,1,1,1,0,0,0,1,1,1,1,1,1,1,0,0,0,] * 2,
            [0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,0,1,1,0,1,0,0,0,0,0,0,1,1,1,0,] * 2,
            [0,0,0,1,0,1,1,0,1,1,0,1,1,1,0,0,0,0,0,1,0,0,1,1,1,1,1,0,0,0,] * 2,
            [0,0,0,1,0,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,1,0,1,0,1,1,] * 2,
            [0,0,0,1,0,1,0,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,1,0,1,0,1,0,0,0,] * 2,
            [0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,1,1,0,] * 2,
            [0,0,0,1,0,1,1,1,1,0,1,0,0,1,1,1,0,1,1,1,1,0,1,1,1,0,1,0,0,0,] * 2,
            [0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,1,1,] * 2,
            [0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,] * 2,
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,] * 2,]
    
    start = (0, 0)
    end = (len(map)-1, len(map[0])-1)

    path = astar(map, start, end)

    if print_map:
      for step in path:
        map[step[0]][step[1]] = 2
      
      for row in map:
        line = []
        for col in row:
          if col == 1:
            line.append("\u2588")
          elif col == 0:
            line.append(" ")
          elif col == 2:
            line.append(".")
        print("".join(line))

    print(path)
example()
#map=Map_Obj(task=1)
#print(map.print_map(map))