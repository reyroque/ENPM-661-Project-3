# Project 2: Implementation of the Dijkstra Algorithm for a Point Robot
# Github Repository: https://github.com/reyroque/ENPM661-DijkstraPointRobot
# Written by: Rey Roque-Pérez
# UID: 120498626

from queue import PriorityQueue
import time

import numpy as np
import cv2

class UpdateableQueue:
    def __init__(self):
        self.queue = PriorityQueue()
        self.entry_finder = {}  # Maps items to their corresponding queue entries

    def put(self, priority, item):
        if item in self.entry_finder:
            current_priority, _ = self.entry_finder[item]
            if priority < current_priority:
                # Update the priority in the entry_finder
                self.entry_finder[item] = (priority, item)
                # Update the priority in the queue
                self.queue.put((priority, item))
                #print('Node Replaced in queue')
            #else:
                #print('Node already in queue')
        else:
            entry = (priority, item)
            self.entry_finder[item] = entry
            self.queue.put(entry)
            #print('Node Added to queue')

    def remove(self, item):
        entry = self.entry_finder.pop(item)
        return entry[1]  # Return the item removed from the queue

    def get(self):
        priority, item = self.queue.get()
        if item in self.entry_finder:  # Check if the item is still in entry_finder
            del self.entry_finder[item]  # Remove the item from the entry_finder
        return priority, item

    def empty(self):
        return self.queue.empty()

    def __contains__(self, item):
        return item in self.entry_finder

###### Move functions ######
def move_right(node):
    new_node = (node[0] + 1, node[1])
    return new_node

def move_left(node):
    new_node = (node[0] - 1, node[1])
    return new_node

def move_up(node):
    new_node = (node[0], node[1] + 1)
    return new_node

def move_down(node):
    new_node = (node[0], node[1] - 1)
    return new_node

def move_up_right(node):
    new_node = (node[0] + 1, node[1] + 1)
    return new_node

def move_up_left(node):
    new_node = (node[0] - 1, node[1] + 1)
    return new_node

def move_down_right(node):
    new_node = (node[0] + 1, node[1] - 1)
    return new_node

def move_down_left(node):
    new_node = (node[0] - 1, node[1] - 1)
    return new_node

def get_straight_moves(node):
    actions = (move_right(node),move_left(node),move_up(node),move_down(node))
    return actions

def get_diagonal_moves(node):
    actions = (move_up_right(node),move_up_left(node),move_down_right(node),move_down_left(node))
    return actions

###### End Move functions ######

###---------------------------------------###

# Check to see if node in question lies within obstacle space
# Return 'False' if in free space, 'True' if in an obstacle or outside the boundaries
# These equations include the 5 space boundary around the obstacles  
def inObstacle(maybeNode):

  node = tuple(maybeNode)
  xnode = node[0]
  ynode = node[1]
  vibes = False

  # check if in map
  if xnode < 5 or xnode > 1195 or ynode < 5 or ynode > 495:
    vibes = True

  # check first obstacle (rectangle)
  elif xnode > 95 and xnode < 180 and ynode > 95:# and ynode <= 500:
    vibes = True

  # check second obstacle (rectangle)
  elif xnode > 270 and xnode < 355 and ynode < 405: # and ynode >= 0
    vibes = True

  # check third obstacle (hexagon)
  elif xnode > 515 and xnode < 785 and (0.556*xnode - ynode + 43.66 > 0) and (-0.556*xnode - ynode + 766.4 > 0) and (-0.556*xnode - ynode + 456.34 < 0) and (0.556*xnode - ynode - 266.4 < 0):
    vibes = True

# The next three compose the concave fourth obstacle
  elif xnode > 895 and xnode < 1025 and ynode > 370 and ynode < 455:
    vibes = True

  elif xnode > 895 and xnode < 1025 and ynode > 45 and ynode < 130:
    vibes = True
  
  elif xnode > 1015 and xnode < 1105 and ynode > 45 and ynode < 455:
    vibes = True

  # return "vibes". False = node is in free space. True = node is out of map or in obstacle.
  return vibes

# Determines the edges of a hexagon of r radius at a given point and angle
def get_hexagon_coordinates(center_coordinate, radius, angle, padding):
    # Define angles for each point of the hexagon
    angles = np.deg2rad(np.arange(0, 360, 60) + angle)
    
    # Calculate x and y coordinates for each angle
    x_coordinates = np.round((radius + padding) * np.cos(angles) + center_coordinate[0])
    y_coordinates = np.round((radius + padding) * np.sin(angles) + center_coordinate[1])
    
    # Combine x and y coordinates into tuples
    coordinates = [(int(x), int(y)) for x, y in zip(x_coordinates, y_coordinates)]
    return coordinates

# Draws start node and end node
def draw_nodes(canvas, start_node, goal_node):
    cv2.rectangle(canvas, (start_node[0] - 4, 500 - start_node[1] - 4), (start_node[0] + 6, 500 - start_node[1] + 6), color=(0, 250, 0), thickness=cv2.FILLED)
    cv2.rectangle(canvas, (goal_node[0] - 4, 500 - goal_node[1] - 4), (goal_node[0] + 6, 500 - goal_node[1] + 6), color=(0, 0, 255), thickness=cv2.FILLED)

# Populates the canvas with the obstacles
def draw_obstacles(canvas, obstacles, video_output):
    for obstacle in obstacles:       
        if len(obstacle) == 2 and obstacle != ((0,0),(1200,500)):
            start_x = obstacle[0][0]
            start_y = 500 - obstacle[0][1]  # Invert y-value
            end_x = obstacle[1][0]
            end_y = 500 - obstacle[1][1]  # Invert y-value
            start_coordinate = (start_x, start_y)
            end_coordinate = (end_x, end_y)
            cv2.rectangle(canvas, pt1=start_coordinate, pt2=end_coordinate, color=(0, 0, 0), thickness=-1)
        elif len(obstacle) == 6:
            polygon = np.array(obstacle)
            polygon = polygon.reshape(-1, 1, 2)
            cv2.fillPoly(canvas, [polygon], color=(0, 0, 0))
    cv2.imshow('Dijkstra', canvas)
    video_output.write(canvas)
    cv2.waitKey(2000)
    return

# Populates and updates the canvas with explored nodes
def draw_explored(canvas, points, video_output):
    count = 0
    for point in points:
        cv2.rectangle(canvas, (point[0], 500-point[1]), (point[0] + 1, 500-point[1] + 1), color=(200, 0, 0), thickness=-1)
        count += 1
        if count % 1000 == 0:
            count = 0
            cv2.imshow('Dijkstra', canvas)
            cv2.waitKey(int(1000 / 120)) 
            video_output.write(canvas)   
    return

# Populates and updates the canvas with path nodes
def draw_path(canvas, path, video_output):
    count = 0
    for point in path:
        cv2.rectangle(canvas, (point[0], 500-point[1]), (point[0] + 1, 500-point[1] + 1), color=(0, 0, 250), thickness=2)
        count += 1
        if count % 5 == 0:
            count = 0
            cv2.imshow('Dijkstra', canvas)
            video_output.write(canvas)
            cv2.waitKey(int(1000 / 120))  
    return

# Adds seconds at end of video
def add_blank_frames(canvas, video_output, fps, seconds):
    blank_frames = fps * seconds
    for _ in range(blank_frames):
        video_output.write(canvas)
    return

# Calls the functions for populating the canvas
def record_animation(obstacles, explored, path, start_node, goal_node):
    # Initialize VideoWriter
    v_writer = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 60
    video_output = cv2.VideoWriter('dijkstra_output.mp4', v_writer, fps, (1200, 500))

    canvas = np.ones((500, 1200, 3), dtype=np.uint8) * 255  # White canvas
    
    draw_nodes(canvas, start_node, goal_node)
    draw_obstacles(canvas, obstacles, video_output)
    add_blank_frames(canvas, video_output, fps, 2)    
    draw_explored(canvas, explored, video_output)
    draw_nodes(canvas, start_node, goal_node)
    draw_path(canvas,path, video_output)
    cv2.waitKey(3000)
    add_blank_frames(canvas, video_output, fps, 2)
    video_output.release()
    return

# Creates a list of the obstacles in the workspace
def obstacle_space():
    obstacle_list = []

    obstacle_list.append(((0,0),(1200,500)))
    obstacle_list.append(((100,100),(175,500)))
    obstacle_list.append(((275,0),(350,400)))
    obstacle_list.append(((900,50),(1100,125)))
    obstacle_list.append(((900,375),(1100,450)))
    obstacle_list.append(((1020,125),(1100,375)))
    
    polygon = (get_hexagon_coordinates((650,250),150,90,0))
    obstacle_list.append(polygon)

    return obstacle_list

def dijkstra_algorithm(start_node, goal_node, obstacles):
    # Create cost_grid and initialize cost to come for start_node
    cost_grid = [[float('inf')] * 500 for _ in range(1200)]
    cost_grid[start_node[0]][start_node[1]] = 0

    # Create grid to store parents
    parent_grid = [[None] * 500 for _ in range(1200)]
    parent_grid[start_node[0]][start_node[1]] = None

    # Create grid to store parents
    visited_grid = [[False] * 500 for _ in range(1200)]

    visited_list = []

    # Priority queue to store open nodes
    # Cost to come, coordinate values (x,y), parent
    open_queue = UpdateableQueue()
    open_queue.put(0, start_node)  # (priority, node)

    while not open_queue.empty():
        _, node = open_queue.get()
        visited_grid[node[0]][node[1]] = True
        visited_list.append(node)

        if node == goal_node:
            return parent_grid, visited_list

        # Get neighboring nodes
        straight_moves = get_straight_moves(node)
        diagonal_moves = get_diagonal_moves(node)
        node_cost = cost_grid[node[0]][node[1]]

        for move in straight_moves:
            if not visited_grid[move[0]][move[1]] and not inObstacle(move):
                new_cost = node_cost + 1
                if new_cost < cost_grid[move[0]][move[1]]:
                    cost_grid[move[0]][move[1]] = new_cost
                    open_queue.put(new_cost, move)
                    parent_grid[move[0]][move[1]] = node

        for move in diagonal_moves:
            if not visited_grid[move[0]][move[1]] and not inObstacle(move):
                new_cost = cost_grid[node[0]][node[1]] + 1.4
                if new_cost < cost_grid[move[0]][move[1]]:
                    cost_grid[move[0]][move[1]] = new_cost
                    open_queue.put(new_cost, move)
                    parent_grid[move[0]][move[1]] = node  

    return print("Failed to find goal")

# Backtracking using path list created from visited/path dictionary
def find_path(parent_grid, start, goal):
    current_node = goal
    path = [current_node]
    while start != current_node:
        current_node = parent_grid[current_node[0]][current_node[1]]
        path.insert(0, current_node)
    return path


#### Main ###
# Get obstacles
obstacles = obstacle_space()
padding = 0

# Get and verify input coordinates
xs = int(input('Enter x coordinate value for start coordinate: '))
ys = int(input('Enter y coordinate value for start coordinate: '))
start_node = tuple((xs, ys))
while inObstacle(start_node):
    print('Node outside workspace or in obstacle. Choose new start location')
    xs = int(input('Enter x coordinate value for start location: '))
    ys = int(input('Enter y coordinate value for start location: '))    
    start_node = tuple((xs, ys))

# Get and verify input coordinates
xg = int(input('Enter x coordinate value for goal coordinate: '))
yg = int(input('Enter y coordinate value for goal coordinate: '))
goal_node = tuple((xg,yg))
while inObstacle(goal_node):
    print('Node outside workspace or in obstacle. Choose new goal location')
    xg = int(input('Enter x coordinate value for goal location: '))
    yg = int(input('Enter y coordinate value for goal location: '))
    goal_node = tuple((xg,yg))

# Start timer
ti = time.time()

print('Exploring nodes...')
explored_dict = dijkstra_algorithm(start_node, goal_node, obstacles)

print('Generating path...')
path = find_path(explored_dict[0], start_node, goal_node)

# Get time taken to find path
tf = time.time()
print('Path found in: ', tf-ti)

record_animation(obstacles, explored_dict[1], path, start_node, goal_node)