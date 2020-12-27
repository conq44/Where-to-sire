import numpy as np
import heapq
import math
import time
from Occupancy_grid import *

MAX_ITERS = 200

class Node:
    def __init__(self, data):
        self.data = data  # stores the node id number
        self.dist = math.inf  # stores distance from start node
        self.prev = None  # cache for the predecessor node in path
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
    # return np.linalg.norm(np.array(node.data) - np.array(end_node.data), ord = np.inf)/


# RRT helper functions


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
    A, B = np.array(l1)
    C, D = np.array(l2)
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def is_free_path(obstacles, v_new, v_nearest):
    """finds if line intersects obstacles"""
    for line in obstacles:
        if is_intersection(line, [v_new, v_nearest]):
            return False
    return True


def get_nearest(v_new, g):
    min_dist = math.inf
    closest_vertex = None
    for vertex in g.keys():
        print(vertex)
        dist = get_distance(vertex, v_new)
        if dist < min_dist:
            closest_vertex = vertex
            min_dist = dist
    return closest_vertex


def get_path(g, new_node, start_pos):
    count = 0
    path = []
    prev_pos = new_node
    while count < MAX_ITERS:
        count += 1
        curr_pos = g[prev_pos]
        path.append([prev_pos, curr_pos])
        prev_pos = curr_pos
        if prev_pos == start_pos:
            return path
    return False


def is_end_region(new_pos, end_pos):
    xn, yn = new_pos
    xe, ye = end_pos
    if xe + 1 >= xn >= xe - 1 and ye + 1 >= yn >= ye -1:
        return True
    else:
        return False


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
    g = {start_pos.data: []}
    while count < 100:
        count += 1
        xn = np.random.randint(grid_obj.lo[0], grid_obj.hi[0])
        yn = np.random.randint(grid_obj.lo[1], grid_obj.hi[1])
        new_node = (xn, yn)
        x_nearest = get_nearest(new_node, g)
        if is_free_path(grid_obj.obstacles, new_node, x_nearest):
            grid_obj.lines.append([new_node, x_nearest])
            # g[x_nearest] += [new_node]
            g[new_node] = x_nearest
            if is_end_region(new_node, end_pos.data):
                print('ha')
                path = get_path(g, new_node, start_pos.data)
                if not path:
                    print('planning_failed: path illegal')
                return g.keys(), new_node, path
    print('planning failed: reached max nodes')
    return False


bottom_left = (0, 0)
top_right = (20, 20)
resolution = 1
# obstacles = [(6, 10), (7, 10), (8, 10), (9, 10), (10, 10), (11, 10), (12, 10), (12, 11), (9, 11), (9, 12)]
# obstacles = []
obstacles = [[(6, 10), (14, 10)], [(14, 10), (14, 12)], [(10, 10), (14, 12)], [(10, 10), (6, 12)], [(6,12), (6, 10)]]
grid = OccupancyGridRRT(bottom_left, top_right, resolution, obstacles)
start_pos = Node((0, 0))
start_pos.dist = 0
start_pos.direction = 90
end_pos = Node((10, 12))
# start_time_2 = time.time()
# path, discovered = Astar_search(start_pos, end_pos, grid)
# print("--- %s seconds for a star ---" % (time.time() - start_time_2))
# grid.plot_grid(start_pos.data, end_pos.data, path, discovered)
# start_time = time.time()
# path2, discovered2 = dijkstra(start_pos, end_pos, grid)
# print("--- %s seconds for dijkstra ---" % (time.time() - start_time))
# grid.plot_grid(start_pos.data, end_pos.data, path2, discovered2)
# # print(get_h_cost(Node((2,2)),Node((4,4))))
discovered, last_node, path = RRT(start_pos, end_pos, grid)
print(last_node)
grid.plot_grid(start_pos.data, end_pos.data, discovered, path)
