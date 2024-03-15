import numpy as np
import cv2


# Take current node from open list and make 8 new nodes and 8 new costs
def newNodes(nodeState, Goal, r):

  # Extract current node information
  node = tuple(nodeState)
  x = node[0]
  y = node[1]
  theta = node[2]

  # Extract goal node information
  goal = tuple(Goal)
  goalx = goal[0]
  goaly = goal[1]
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
  for i in range (-2,3):
    if theta + 30* i > 330:
      thetalist.append(theta + 30*i - 360)
    else: 
      thetalist.append(theta + 30*i)
    dx = r*np.cos(np.pi*thetalist[i+2]/180)
    dy = r*np.sin(np.pi*thetalist[i+2]/180)
    newX = round((x+dx)*2)/2
    newY = round((y+dy)*2)/2
    c2c = round(np.sqrt(np.square(x - newX) + np.square(y - newY)), 2)
    if thetalist[i+2] < 0:
      thetalist[i+2] = thetalist[i+2] + 360
    theta = thetalist[i+2]//30
    newNodes.append((c2c, (newX, newY, theta)))
    # newNodes.append((c2c, (newX, newY, thetalist[i+2])))
    #c2c.append(round(np.sqrt(np.square(x - round((x + dx)*2)/2) + np.square(y - round((y+dy)*2)/2)), 2))
    #c2g.append(round(np.sqrt(np.square(goalx - newX) + np.square(goaly - newY)), 2))

  return newNodes

# test starting node
node = (.5,.5,0)

# test goal node
goal = (0, 0, 0)

# Step Size
r = 2.0

# Test function
#newNodes, c2c, c2g] = newNodes(node, goal)
newNodes = newNodes(node, goal, r)
print('')
print(newNodes)
print('')

def heuristic(node, goal_node, weight):
  return weight * np.sqrt(np.square(goal_node[0] - node[0]) + np.square(goal_node[1] - node[1]))
  
def getRecPoints(currentNode):

  # coordinates of center of circle and orientation
  xc = currentNode[0]
  yc = currentNode[1]
  theta = currentNode[2]

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

##### TEST #####

currentNode = (10, 10, 45)

green = (31, 80, 12)

# Make white canvas
map = np.ones((500,1200,3), dtype=np.uint8) # blank map
map = 255*map # make it white

points = getRecPoints(currentNode)
points = points.reshape(-1,1,2)
cv2.fillPoly(map, np.int32([points]), green)
#cv2.polylines(map, np.int32([points]), isClosed=True,thickness=1,color=green)

cv2.drawMarker(map, (currentNode[0],500-currentNode[1]), color = green, thickness=2, markerType= cv2.MARKER_SQUARE, line_type=cv2.LINE_AA, markerSize=1)

cv2.imshow("map", map)
cv2.waitKey(0)