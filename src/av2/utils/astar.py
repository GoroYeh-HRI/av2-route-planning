import numpy as np

""" Functions for A* planning """
class AstarNode:
    def __init__(self, lsid, position, parent_node=None):
        self.lsid = lsid           # lane segment id
        self.position = position   # mean position (x, y)
        self.parent = parent_node  # pointer to parent_node
        self.g = 0 
        self.h = 0
        self.f = 0
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f
        
    def __hash__(self):
        return hash(self.position) 

def calc_ls_position(avmap, ls_id):
    """Calculate mean position for a lane segment    """
    ls_centerline = avmap.get_lane_segment_centerline(ls_id)
    xs = ls_centerline[:,0]
    ys = ls_centerline[:,1]
    ls_pos = (np.mean(xs), np.mean(ys))
    return ls_pos

import heapq


def astar(avmap, start_node, goal_node):
    """
    avmap: ArgoverseStaticMap
    start_node: start node
    goal_node : goal node

    """
    open_list = []
    closed_set = set()
    
    heapq.heappush(open_list, start_node)
    final_node = None

    while open_list:

        current_node = heapq.heappop(open_list)
        final_node = current_node

        if current_node == goal_node:

            path = [] # a list of node.lsid
            while current_node:
                path.append(current_node.lsid)
                current_node = current_node.parent
            return path[::-1]
        
        closed_set.add(current_node)
        
        for neighbor in get_neighbors(avmap, current_node):
            if neighbor in closed_set:
                continue
            
            neighbor.g = current_node.g + 1 # 1 -> distance from cur to neighbor
            neighbor.h = heuristic(neighbor, goal_node)
            neighbor.f = neighbor.g + neighbor.h
            
            if neighbor not in open_list:
                heapq.heappush(open_list, neighbor)
            else:
                for open_node in open_list:
                    if open_node == neighbor and open_node.g > neighbor.g:
                        open_node.g = neighbor.g
                        open_node.parent = neighbor.parent
    

    print(f"Astar failed. Not able to find the path")
    # return from what we have
    path = [] # a list of node.lsid
    while final_node:
        path.append(final_node.lsid)
        final_node = final_node.parent
    return path[::-1]


# TODO: Instead of 4 direction, find neighbors from ls.successors/right_neighbor_id/left_neighbor
def get_neighbors(avmap, node):
    neighbors = []

    suc_ids = avmap.get_lane_segment_successor_ids(node.lsid)
    left_id = avmap.get_lane_segment_left_neighbor_id(node.lsid)
    right_id = avmap.get_lane_segment_right_neighbor_id(node.lsid) 

    # ls.successors
    for successor_id in suc_ids:
        if successor_id in avmap.vector_lane_segments:
            pos = calc_ls_position(avmap, successor_id)
            neighbors.append(AstarNode(successor_id, pos, node))


    # left neighbor
    if left_id:
        if left_id in avmap.vector_lane_segments:
            pos = calc_ls_position(avmap, left_id)
            neighbors.append(AstarNode(left_id, pos, node))

    # right neighbor
    if right_id:
        if right_id in avmap.vector_lane_segments:        
            pos = calc_ls_position(avmap, right_id)
            neighbors.append(AstarNode(right_id, pos, node))                
    
    return neighbors

# TODO: Distance metric might need to be adjusted
def heuristic(node, end):
    return abs(node.position[0] - end.position[0]) + abs(node.position[1] - end.position[1])