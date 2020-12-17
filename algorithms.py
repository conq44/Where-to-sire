import numpy as np
import heapq
import math
from Occupancy_grid import *


class Node:
    def __init__(self, data):
        self.data = data  # stores the node id number
        self.dist = math.inf  # stores distance from start node
        self.prev = None  # cache for the predecessor node in path

    def __repr__(self):
        return f'Node data: {self.data}'

    def __lt__(self, other):
        return self.dist < other.dist


def dijkstra(start_pos, end_pos, grid_obj):
    queue = [start_pos]
    path = []
    discovered = set()
    while len(queue) > 0:
        curr_pos = heapq.heappop(queue)
        if curr_pos.data not in discovered:
            print(curr_pos)
            discovered.add(curr_pos.data)
            if curr_pos.data == end_pos.data:
                path.append(curr_pos.data)
                while curr_pos.data != start_pos.data:
                    previous = curr_pos.prev  # add nodes to the path starting from end_node
                    path.append(previous.data)
                    curr_pos = previous
                print(path)
                return path
            neighbors = grid_obj.get_neighbors(curr_pos.data)
            for pos in neighbors:
                pos_node = Node(pos)
                pos_node.dist = curr_pos.dist + 1
                pos_node.prev = curr_pos
                heapq.heappush(queue, pos_node)
        else:
            print('ha')
            continue
    return False

bottom_left = (0,0)
top_right = (20,20)
resolution = 1
obstacles = [(1,5),(2,5),(3,5),(4,5),(5,5),(6,5),(7,5),(7,6)]
grid = OccupancyGrid(bottom_left, top_right, resolution, obstacles)
start_pos = Node((0,0))
start_pos.dist = 0
path = dijkstra(start_pos, Node((5,7)), grid)
grid.plot_grid(start_pos.data, (5,7), path)