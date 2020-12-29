import numpy as np
import heapq
import math
import time
from Occupancy_grid import *

MAX_ITERS = 2000


class Node:
    def __init__(self, data):
        self.data = data  # stores the node id number
        self.dist = math.inf  # stores distance from start node
        self.prev = None  # cache for previous node in path
        self.direction = None  # stores the direction taken to reach this node

    def __repr__(self):
        return f'Node data: {self.data}'

    def __lt__(self, other):
        return self.dist < other.dist


def get_dir_penalty(curr_dir, new_dir):
    """gets the penalty for change in direction. Greater the angle change
    larger the penalty. Prevents zig-zag paths."""
    change = abs(curr_dir - new_dir)
    if change == 0:
        penalty = 0
    elif change in [1, 7]:
        penalty = 0.1
    elif change in [2, 6]:
        penalty = 0.15
    elif change in [3, 5]:
        penalty = 0.2
    elif change == 4:
        penalty = 0.25
    else:
        return False
    return penalty


def get_h_cost(node, end_node):
    """returns the heuristic cost.Here it is the diagonal distance"""
    dx = abs(node.data[0] - end_node.data[0])
    dy = abs(node.data[1] - end_node.data[1])
    D = 1
    D2 = 1
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)


# RRT HELPER FUNCTIONS


def get_distance(node, end_pos):
    """returns the heuristic cost.Here it is the diagonal distance"""
    dx = abs(node[0] - end_pos[0])
    dy = abs(node[1] - end_pos[1])
    D = 1
    D2 = 1
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)


def ccw(A, B, C):
    return np.cross(B - A, C - A) > 0


def is_intersection(l1, l2):
    """ finds if two lines intersect """
    A, B = np.array(l1)
    C, D = np.array(l2)
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def is_free_path(obstacles, v_new, v_nearest):
    """finds if there is a free path between two nodes """
    for line in obstacles:
        if is_intersection(line, [v_new, v_nearest]):
            return False
    return True


def get_nearest(new_node, parents):
    """ gets the node in the tree closest to new_node """
    min_dist = math.inf
    closest_vertex = None
    for vertex in parents.keys():
        dist = get_distance(vertex, new_node)
        if dist < min_dist:
            closest_vertex = vertex
            min_dist = dist
    return closest_vertex


def get_path(parents, new_node, start_pos):
    """ Returns path by iterating backwards from the last node """
    count = 0
    path = [new_node]
    prev_pos = new_node
    while count < MAX_ITERS:
        count += 1
        curr_pos = parents[prev_pos]
        path.append(curr_pos)
        prev_pos = curr_pos
        if prev_pos == start_pos:
            return path
    return False


def is_end_region(new_pos, end_pos):
    """ Success criteria for RRT  """
    xn, yn = new_pos
    xe, ye = end_pos
    if xe + 1 >= xn >= xe - 1 and ye + 1 >= yn >= ye -1:
        return True
    else:
        return False


def get_shortcut(path, obstacles):
    """ finds a shorter path by removing redundant nodes """
    n_path = [node for node in path]
    x3 = n_path[2]
    i = 0
    while x3 != n_path[-1]:
        make_shorter = True
        while make_shorter:
            x1 = n_path[i]
            x2 = n_path[i+1]
            x3 = n_path[i+2]
            if x3 == n_path[-1]:
                make_shorter = False
            if is_free_path(obstacles, x1, x3):
                del n_path[i+1]
            else:
                i += 1
                make_shorter = False
    return n_path


# END OF HELPER FUNCTIONS

def dijkstra(start_pos, end_pos, grid_obj):
    """dijkstra path planning algorithm, returns shortest path"""
    queue = [start_pos]  # a heap that keeps track of closest nodes
    path = []  # stores the path from start to end
    discovered = set()  # set of already visited nodes
    while len(queue) > 0:
        curr_pos = heapq.heappop(queue)  # pops minimum distance node
        if curr_pos.data not in discovered:
            curr_dir = curr_pos.direction
            discovered.add(curr_pos.data)
            if curr_pos.data == end_pos.data:
                path.append(curr_pos.data)
                print(curr_pos.dist)
                while curr_pos.data != start_pos.data:
                    previous = curr_pos.prev  # add nodes to the path starting from end_node
                    path.append(previous.data)
                    curr_pos = previous
                return path, discovered
            neighbors, neighbor_directions = grid_obj.get_neighbors(curr_pos.data)
            for i, pos in enumerate(neighbors):
                pos_node = Node(pos)
                pos_node.direction = neighbor_directions[i]
                penalty = get_dir_penalty(curr_dir, pos_node.direction)
                pos_node.dist = curr_pos.dist + 1 + penalty  # total distance cost for a node/grid cell
                pos_node.prev = curr_pos
                heapq.heappush(queue, pos_node)
        else:
            continue
    return False


def Astar_search(start_pos, end_pos, grid_obj):
    queue = [start_pos]  # a heap that keeps track of closest nodes
    path = []  # stores the path from start to end
    discovered = set()  # set of already visited nodes
    while len(queue) > 0:
        curr_pos = heapq.heappop(queue)  # pops minimum distance node
        if curr_pos.data not in discovered:
            curr_dir = curr_pos.direction
            discovered.add(curr_pos.data)
            if curr_pos.data == end_pos.data:
                path.append(curr_pos.data)
                print(curr_pos.dist)
                while curr_pos.data != start_pos.data:
                    previous = curr_pos.prev  # add nodes to the path starting from end_node
                    path.append(previous.data)
                    curr_pos = previous
                return path, discovered
            neighbors, neighbor_directions = grid_obj.get_neighbors(curr_pos.data)
            for i, pos in enumerate(neighbors):
                pos_node = Node(pos)
                pos_node.direction = neighbor_directions[i]
                penalty = get_dir_penalty(curr_dir, pos_node.direction)
                h_cost = get_h_cost(pos_node, end_pos)  # additional penalty; diagonal distance to the end_node
                pos_node.dist = curr_pos.dist + 1 + penalty + h_cost  # total distance cost for a node/grid cell
                pos_node.prev = curr_pos
                heapq.heappush(queue, pos_node)
        else:
            continue
    return False
    pass


def RRT(start_pos, end_pos, grid_obj):
    count = 0
    parents = {start_pos.data: []}
    discovered = set()
    while count < MAX_ITERS:
        count += 1
        xn = np.random.randint(grid_obj.lo[0], grid_obj.hi[0])
        yn = np.random.randint(grid_obj.lo[1], grid_obj.hi[1])
        new_node = (xn, yn)
        if new_node not in discovered:
            x_nearest = get_nearest(new_node, parents)
            if is_free_path(grid_obj.obstacles, new_node, x_nearest):
                grid_obj.lines.append([new_node, x_nearest])
                parents[new_node] = x_nearest
                discovered.add(new_node)
                if is_end_region(new_node, end_pos.data):
                    # print(parents)
                    path = get_path(parents, new_node, start_pos.data)
                    if not path:
                        print('planning_failed: path illegal')
                    return list(discovered), new_node, path
    print('planning failed: reached max nodes')
    return False