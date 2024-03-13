import numpy as np

# Take current node from open list and make 8 new nodes and 8 new costs
def newNodes(nodeState, Goal):

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
  r = 0.5
  newNodes = [] # new node information
  thetalist = [] # list of angles for calculations from 0 deg to 330 deg
  dx = [] # list of changes in x direction
  dy = [] # list of changes in y direction
  c2c = [] # list of costs to come for each new node from parent node
  c2g = [] # list of costs to go for each new node to the goal node

  i = 0

  # For all 12 new nodes, calculate new angle (0 -> 330 deg), the dx and dy values, and add them to the newNodes list
  # Then calculate C2C and C2G for each new node using distance between two points calculation
  for i in range (0,12):
    if theta + 30* i > 330:
      thetalist.append(theta + 30*i - 360)
    else: 
      thetalist.append(theta + 30*i)
    dx = r*np.cos(np.pi*thetalist[i]/180)
    dy = r*np.sin(np.pi*thetalist[i]/180)
    newNodes.append((round((x+dx)*2)/2, round((y+dy)*2)/2, thetalist[i]))
    c2c.append(round(np.sqrt(np.square(x - round((x + dx)*2)/2) + np.square(y - round((y+dy)*2)/2)), 2))
    c2g.append(round(np.sqrt(np.square(goalx - round((x + dx)*2)/2) + np.square(goaly - round((y+dy)*2)/2)), 2))

  return newNodes, c2c, c2g

# test starting node
node = (1,1,0)

# test goal node
goal = (0, 0, 0)

# Test function
[newNodes, c2c, c2g] = newNodes(node, goal)

print('')
print(newNodes)
print('')
print(c2c)
print('')
print(c2g)
print('')