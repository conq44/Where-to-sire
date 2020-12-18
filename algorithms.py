import numpy as np
import heapq
import math
from Occupancy_grid import *


class Node:
    def __init__(self, data):
        self.data = data  # stores the node id number
        self.dist = math.inf  # stores distance from start node
        self.prev = None  # cache for the predecessor node in path
        self.direction = None

    def __repr__(self):
        return f'Node data: {self.data}'

    def __lt__(self, other):
        return self.dist < other.dist

# possible directions = [N S E W NE NW SE SW]
# remember current direction
# add direction change required from each node
# penalize direction change


def get_dir_penalty(curr_dir, new_dir):
    change = abs(curr_dir - new_dir)
    if change == 0:
        penalty = 0
    elif change in [1,7]:
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


def dijkstra(start_pos, end_pos, grid_obj):
    queue = [start_pos]
    path = []
    discovered = set()
    while len(queue) > 0:
        curr_pos = heapq.heappop(queue)
        if curr_pos.data not in discovered:
            # print(curr_pos)
            curr_dir = curr_pos.direction
            discovered.add(curr_pos.data)
            if curr_pos.data == end_pos.data:
                path.append(curr_pos.data)
                while curr_pos.data != start_pos.data:
                    previous = curr_pos.prev  # add nodes to the path starting from end_node
                    path.append(previous.data)
                    curr_pos = previous
                print(path)
                return path
            neighbors, neighbor_directions = grid_obj.get_neighbors(curr_pos.data)
            for i,pos in enumerate(neighbors):
                pos_node = Node(pos)
                pos_node.direction = neighbor_directions[i]
                print(pos_node.direction)
                penalty = get_dir_penalty(curr_dir, pos_node.direction)
                pos_node.dist = curr_pos.dist + 1 + penalty
                pos_node.prev = curr_pos
                heapq.heappush(queue, pos_node)
        else:
            continue
    return False

bottom_left = (0,0)
top_right = (20,20)
resolution = 1
obstacles = [(1,5),(2,5),(3,5),(4,5),(5,5),(6,5),(7,5),(7,6), (4,5), (4,6), (4,7)]
grid = OccupancyGrid(bottom_left, top_right, resolution, obstacles)
start_pos = Node((0,0))
start_pos.dist = 0
start_pos.direction = 90
path = dijkstra(start_pos, Node((5,7)), grid)
grid.plot_grid(start_pos.data, (5,7), path)