from enum import Enum
from queue import PriorityQueue
import numpy as np
from bresenham import bresenham
from sklearn.neighbors import KDTree


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

    return grid, int(north_min), int(east_min)

# Assume all actions cost the same.


class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NW = (-1, -1, np.sqrt(2))
    NE = (-1, 1, np.sqrt(2))
    SW = (1, -1, np.sqrt(2))
    SE = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    # new actions to go diaganolly
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NE)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SE)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NW)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SW)

    return valid_actions


def a_star(grid, h, start, goal):
    print("Trying to get from", start, "to", goal)
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def safe_bres(p1, p2, grid):
    sub_path = list(bresenham(p1[0], p1[1], p2[0], p2[1]))
    # print("bresenham between {0} and {1} for {2} points".format(p1, p2, len(sub_path)))
    for p in sub_path:
        if (grid[p] == 1):
            # print("collision at", p)
            return False
    return True


def bresify_path(path, grid):
    kept = [path[0]]
    last_safe = path[0]
    last_added = path[0]
    for p in path:
        if (p == last_safe):
            continue
        if (safe_bres(last_added, p, grid)):
            last_safe = p
        else:
            kept.append(last_safe)
            last_added = last_safe

    # include the destination
    kept.append(path[-1])

    return kept


def find_closest_safe(point, altitude, safe_points, tree):
    point_2d = (point[0], point[1])
    # query with kdtree for fast lookup of closest item
    idx = tree.query([point_2d], k=1, return_distance=False)[0]
    safe_point = safe_points[idx[0]]
    print("Found {0} which was {1}".format(idx, safe_point))
    return (int(safe_point[0]), int(safe_point[1]))


def plan_takeoff_and_landing(start_point, end_point, altitude, grid):
    safe_points = np.transpose(np.nonzero(grid == 0))
    tree = KDTree(safe_points)

    close_to_start = find_closest_safe(start_point, altitude, safe_points, tree)
    close_to_end = find_closest_safe(end_point, altitude, safe_points, tree)
    close_to_start_2d = (close_to_start[0], close_to_start[1])
    # print("close to start", start_point, close_to_start, grid[close_to_start_2d])
    close_to_end_2d = (close_to_end[0], close_to_end[1])
    # print("close to end", end_point, close_to_end, grid[close_to_end_2d])
    starting_points = [(start_point[0], start_point[1], start_point[2] + altitude, 0),
                       (close_to_start[0], close_to_start[1], start_point[2] + altitude)]
    ending_points = [(close_to_end[0], close_to_end[1], end_point[2] + altitude),
                     (end_point[0], end_point[1], end_point[2] + altitude),
                     (end_point[0], end_point[1], end_point[2])]
    return starting_points, ending_points, close_to_start_2d, close_to_end_2d
