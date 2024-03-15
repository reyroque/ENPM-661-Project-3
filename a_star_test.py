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

###---------------------------------------###
# Check to see if node in question lies within obstacle space
# Return 'False' if in free space, 'True' if in an obstacle or outside the boundaries
# These equations include the 5 space boundary around the obstacles  
def inObstacle(maybeNode):

  node = tuple(maybeNode)
  xnode = node[0]
  ynode = node[1]
  vibes = False
  padding = 5
  radius = 5

  # check if in map
  if xnode < 5 + radius or xnode > 1195 - radius or ynode < 5 + radius or ynode > 495 - radius:
    vibes = True

  # check first obstacle (rectangle)
  elif xnode > 95 - radius and xnode < 180 + radius and ynode > 95 - radius:# and ynode <= 500:
    vibes = True

  # check second obstacle (rectangle)
  elif xnode > 270 - radius and xnode < 355 + radius and ynode < 405 + radius: # and ynode >= 0
    vibes = True

  # check third obstacle (hexagon)
  elif xnode > 515 - radius and xnode < 785 + radius and (0.556*xnode - ynode + 43.66 > 0) and (-0.556*xnode - ynode + 766.4 > 0) and (-0.556*xnode - ynode + 456.34 < 0) and (0.556*xnode - ynode - 266.4 < 0):
    vibes = True

# The next three compose the concave fourth obstacle
  elif xnode > 895 - radius and xnode < 1025 + radius and ynode > 370 - radius and ynode < 455 + radius:
    vibes = True

  elif xnode > 895 - radius and xnode < 1025 + radius and ynode > 45 - radius and ynode < 130 + radius:
    vibes = True
  
  elif xnode > 1015 - radius and xnode < 1105 + radius and ynode > 45 - radius and ynode < 455 + radius:
    vibes = True

  # return "vibes". False = node is in free space. True = node is out of map or in obstacle.
  return vibes

def inGoal(node, goal_node):
    goal_radius = 3
    x_goal = goal_node[0]
    y_goal = goal_node[1]
    theta_goal = goal_node[2]
    x_node = node[0]
    y_node = node[1]
    theta_node = node[2]
    return np.sqrt(np.square(x_node-x_goal) + np.square(y_node-y_goal)) < goal_radius

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

def getRecPoints(currentNode):

  # coordinates of center of circle and orientation
  xc = currentNode[0]
  yc = currentNode[1]
  theta = currentNode[2]*30

  # square side length
  s = 5.0

  # top right
  x1 = xc + s*(np.cos(theta) - np.sin(theta))
  y1 = 500 - yc + s*(np.sin(theta) + np.cos(theta))

  # top left
  x2 = xc + s*(-np.cos(theta) - np.sin(theta))
  y2 = 500 - yc + s*(-np.sin(theta) + np.cos(theta))

  # bottom left
  x3 = xc + s*(-np.cos(theta) + np.sin(theta))
  y3 = 500 - yc + s*(-np.sin(theta) - np.cos(theta))

  # bottom right
  x4 = xc + s*(np.cos(theta) + np.sin(theta))
  y4 = 500 - yc + s*(np.sin(theta) - np.cos(theta))

  coords = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])

  return coords

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
    cv2.imshow('A*', canvas)
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
            cv2.imshow('A*', canvas)
            cv2.waitKey(int(1000 / 120)) 
            video_output.write(canvas)   
    return

# Populates and updates the canvas with path nodes
def draw_path(canvas, path, video_output):
    count = 0
    for point in path:
        # Draw the path node
        cv2.rectangle(canvas, (point[0], 500 - point[1]), (point[0] + 1, 500 - point[1] + 1), color=(0, 0, 250), thickness=2)

        # Calculate square points
        square_points = getRecPoints(point)
        square_points = square_points.reshape(-1, 1, 2).astype(np.int32)  # Convert to int32

        # Create a temporary copy of the canvas
        temp_canvas = canvas.copy()

        # Draw new square on the temporary canvas
        cv2.fillPoly(temp_canvas, [square_points], (0, 250, 0))

        count += 1
        if count % 1 == 0:
            count = 0
            cv2.imshow('A*', temp_canvas)
            video_output.write(temp_canvas)
            cv2.waitKey(int(1000 / 60))

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
    video_output = cv2.VideoWriter('a_star_output.mp4', v_writer, fps, (1200, 500))

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

###### Move functions ######
def newNodes(nodeState, Goal, r):
    # Extract current node information
    node = tuple(nodeState)
    x = node[0]/2
    y = node[1]/2
    
    # Converting index to angle between 0 - 330
    theta = node[2]*30

    # Extract goal node information
    goal = tuple(Goal)
    goalx = goal[0]/2
    goaly = goal[1]/2
    goaltheta = goal[2]

    # define distance (r), and make empty lists to fill with values
    newNodes = [] # new node information
    thetalist = [] # list of angles for calculations from 0 deg to 330 deg
    dx = [] # list of changes in x direction
    dy = [] # list of changes in y direction
    c2c = [] # list of costs to come for each new node from parent node
    c2g = [] # list of costs to go for each new node to the goal node

    i = 0

    # For all 12 new nodes, calculate new angle (0 -> 330 deg), the dx and dy values, and add them to the newNodes list
    # Then calculate C2C and C2G for each new node using distance between two points calculation
    for i in range(-2, 3):
        new_theta = (theta + 30 * i) % 360  # Ensure angle stays within range 0 to 360 degrees
        thetalist.append(new_theta)
        dx = r * np.cos(np.pi * new_theta / 180)
        dy = r * np.sin(np.pi * new_theta / 180)
        newX = round((x + dx) * 2) / 2
        newY = round((y + dy) * 2) / 2
        c2c = round(np.sqrt(np.square(x - newX) + np.square(y - newY)), 2)
        newNodes.append((c2c*2, (int(newX * 2), int(newY * 2), new_theta//30)))
    return newNodes
###### End Move functions ######

def heuristic(node, goal_node, weight):
  return weight * np.sqrt(np.square(goal_node[0] - node[0]) + np.square(goal_node[1] - node[1]))

def a_star_algorithm(start, goal, obstacles, weight):
    start_node = (int(start[0]*2), int(start[1]*2), start[2])
    goal_node = (int(goal[0]*2), int(goal[1]*2), goal[2])

    # Create cost_grid and initialize cost to come for start_node
    cost_grid = [[[float('inf')] * 12 for _ in range(500)] for _ in range(1200)]
    cost_grid[start_node[0]][start_node[1]][start_node[2]] = 0

    # Create grid to store parents
    parent_grid = [[[None] * 12 for _ in range(500)] for _ in range(1200)]
    parent_grid[start_node[0]][start_node[1]][start_node[2]] = None

    # Create grid to store parents
    visited_grid = [[[False] * 12 for _ in range(500)] for _ in range(1200)]
    visited_list = []

    # Priority queue to store open nodes
    # Cost to come, coordinate values (x,y), parent
    open_queue = UpdateableQueue()
    open_queue.put(0, start_node)  # (priority, node)

    while not open_queue.empty():
        _, node = open_queue.get()
        visited_grid[node[0]][node[1]][node[2]] = True
        visited_list.append(node)

        if inGoal(node, goal_node):
            return parent_grid, visited_list

        # Get neighboring nodes
        step = 0.5
        actions = newNodes(node, goal_node, step)
        node_cost = cost_grid[node[0]][node[1]][node[2]]

        for action in actions:
            # action = (c2c,(x,y,theta))
            action_cost = action[0]
            move = action[1]
            if not visited_grid[move[0]][move[1]][move[2]] and not inObstacle(move):
                new_cost = node_cost + action_cost
                if new_cost < cost_grid[move[0]][move[1]][move[2]]:
                    cost_grid[move[0]][move[1]][move[2]] = new_cost
                    priority = new_cost + heuristic(move, goal_node, weight)
                    open_queue.put(priority, move)                    
                    parent_grid[move[0]][move[1]][move[2]] = node

    return parent_grid, visited_list, print("Failed to find goal")

# Backtracking using path list created from visited/path dictionary
def find_path(parent_grid, visited_list, start):
    current_node = visited_list[-1]
    path = [current_node]
    start_node = start
    while start_node != current_node:
        #print(current_node)
        temp_node = parent_grid[int(current_node[0])][int(current_node[1])][current_node[2]]
        current_node = temp_node
        path.insert(0, current_node)
    #print(path)
    return path

#### Main ###
# Get obstacles
obstacles = obstacle_space()
padding = 0

# # Get and verify input coordinates
# xs = int(input('Enter x coordinate value for start coordinate: '))
# ys = int(input('Enter y coordinate value for start coordinate: '))
# thetas = int(input('Enter theta value for start coordinate: '))
# start = tuple((xs, ys, thetas))
# while inObstacle(start_node):
#     print('Node outside workspace or in obstacle. Choose new start location')
#     xs = int(input('Enter x coordinate value for start location: '))
#     ys = int(input('Enter y coordinate value for start location: '))
#     thetas = int(input('Enter theta value for start coordinate: '))      
#     start = tuple((xs, ys, thetas))

# # Get and verify input coordinates
# xg = int(input('Enter x coordinate value for goal coordinate: '))
# yg = int(input('Enter y coordinate value for goal coordinate: '))
# thetag = int(input('Enter theta value for goal coordinate: '))
# goal = tuple((xg, yg, thetag))
# while inObstacle(goal_node):
#     print('Node outside workspace or in obstacle. Choose new goal location')
#     xg = int(input('Enter x coordinate value for goal location: '))
#     yg = int(input('Enter y coordinate value for goal location: '))
#     thetag = int(input('Enter theta value for goal coordinate: '))
#     goal = tuple((xg, yg, thetag))

start_node = (6, 6, 6)
goal_node = (12, 125, 2)

start = (int(start_node[0]*2), int(start_node[1]*2), start_node[2])
goal = (int(goal_node[0]*2), int(goal_node[1]*2), goal_node[2])

# Start timer
ti = time.time()

print('Exploring nodes...')
explored = a_star_algorithm(start_node, goal_node, obstacles, 2)
parent_grid = explored[0]
visited_list = explored[1]

print('Generating path...')
path = find_path(parent_grid, visited_list, start)

# Get time taken to find path
tf = time.time()
print('Path found in: ', tf-ti)
print(path)
record_animation(obstacles, visited_list, path, start, goal)