import numpy as np

class Node:
  def __init__(self, x, y, center):
    self.neighbors = []
    self.x = x # give each node an x and y location
    self.y = y
    self.center = center
    self.curr_cost = float("inf")
    if(x != center): #Set a higher cost for paths not down the center
      self.travel_cost = 5
    else:
      self.travel_cost = 1
    self.previous = None

  def __str__(self):
    return "Point: "+str(self.x)+", "+str(self.y)

#def generateSingleI(obstacleLocations):
#if __name__ =='__main__':
def generateSingleI(obstacleList, center, starty, endy):
  xx, yy = np.meshgrid(np.arange(1, 7.5, .5), np.arange(0, 16, .5))
  my_grid_list = list(zip(xx.flatten(), yy.flatten()))
  my_node_list = [Node(point[0], point[1], center) for point in my_grid_list]

  for node in my_node_list:
    #print("Node: "+str(node))
    #print("Neighbors: ")
    # Add the 8-connected neighbors
    for test_node in my_node_list:
      if test_node.x==node.x and (test_node.y==node.y+.5 or test_node.y==node.y-.5):
        node.neighbors.append(test_node)
      if test_node.y==node.y and (test_node.x==node.x+.5 or test_node.x==node.x-.5):
        node.neighbors.append(test_node)
      if (test_node.x==node.x-.5 or test_node.x==node.x+.5) and (test_node.y==node.y+.5 or test_node.y==node.y-1/2):
        node.neighbors.append(test_node)

  # Make a dictionary for quick access by point
  my_node_dict = dict(zip(my_grid_list, my_node_list))
  #print(my_node_dict.keys())

  startNode = (center,starty)
  endNode = (center,endy)
  #print startNode, endNode
  start_node = my_node_dict[startNode]
  end_node = my_node_dict[endNode]
  #start_node = my_node_dict[(2.5,13)]
  #end_node = my_node_dict[(2.5,3)]
  # Setup some basic obstacles
##  my_node_dict[(0,4)].travel_cost = float("inf")
##  my_node_dict[(1,5)].travel_cost = float("inf")
##  my_node_dict[(-1,5)].travel_cost = float("inf")
  for obstacle in obstacleList:
    obX = obstacle[0]
    obY = obstacle[1]
    for i in np.arange(-.5,1,.5):
      for j in np.arange(-.5,1,.5):
        #print(str(obX+i)+", "+str(obY+j))
        #print(str(center)+", " + str(starty))
        try:
          if (float(obX)+i == center) and (float(obY)+j == starty):
            startNode = (center+1, starty)
            #print(str(startNode))
            #print('hello')
            start_node = my_node_dict[startNode]
          if (float(obX)+i == center) and (float(obY)+j == endy):
            endNode = (center+1, endy)
            end_node = my_node_dict[endNode]
          my_node_dict[(obX+i,obY+j)].travel_cost = float("inf")
        except:
          continue

  # Setup the unevaluated and evaluated lists
  unevaluated = my_node_list[:] # Make a copy
  evaluated = []
  my_node_dict[(start_node.x, start_node.y)].curr_cost = 0

  while len(unevaluated) > 0:
    min_node = min(unevaluated, key=lambda x: x.curr_cost)
    my_cost = min_node.curr_cost
    my_node = min_node
    for neighbor in my_node.neighbors:
      neighbor_node = neighbor
      neighbor_cost = neighbor.travel_cost
      if neighbor_node.curr_cost > my_cost + neighbor_cost:
        neighbor_node.curr_cost = my_cost+neighbor_cost
        neighbor.previous = my_node
    # MOve this node to the evaluated list
    unevaluated.remove(min_node)
    evaluated.append(min_node)

  # Now that all of the nodes have been evaluated, unwind the stack
  curr_node = my_node_dict[(end_node.x,end_node.y)]  # Start at the end
  path = []
  while curr_node != None:
    path.append(curr_node)
    curr_node = curr_node.previous

  # Get the order from start to end
  path.reverse()
  return path
     
def generateTripleI(obstacles):
  FullPath = []
  path_str = "Path: "
  #This assumes our plow is angled to the right.
  centerPathUp = generateSingleI(obstacles, 3.5, 3.0, 13.0)
  FullPath.extend(centerPathUp)
  leftPathBack = generateSingleI(obstacles, 2.5, 13.0, 3.0)
  FullPath.extend(leftPathBack)
  rightPathUp = generateSingleI(obstacles, 4.5,3.0,13.0)
  FullPath.extend(rightPathUp)
  BackToGaragePath = generateSingleI(obstacles, 2.5, 13.0, 2.0)
  FullPath.extend(BackToGaragePath)
  return FullPath
