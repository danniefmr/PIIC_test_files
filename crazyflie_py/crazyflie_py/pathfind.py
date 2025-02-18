from PIL import Image, ImageOps
import numpy as np
import random

class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None
    
#RRT alg
class RRTAlgorithm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[0])
        self.nearestNode = None
        self.iterations = min(numIterations, 10)
        self.grid = grid
        self.rho = stepSize
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints= []

    #add node to nearest node and add goal when reached
    def addChild(self, locationX, locationY):
        pass

    #sample a random point within arena limits
    def sampleAPoint(self):
        pass

    #steer a distance stepsize from start to end of location
    def steerToPoint(self, locationStart, locationEnd):
        pass

    #check if obstacle lies between the start and end point of the edge
    def isInObstacle(self, locationStart, locationEnd):
        pass

    #find unit vector between 2 points which form a vector
    def unitVector(self, locationStart, locationEnd):
        pass

    #find nearest node from an unconnected point
    def findNearest(self, root, point):
        pass

    #find euclidean distance between a node and XY point
    def distance(self, node1, point):
        pass
    
    #check if goal has been reached
    def goalFound(self, point):
        pass

    #reset nearestNode and nearest Distance
    def resetNearestValues(self):
        pass

    #trace the path from goal to start
    def retraceRRTPath(self, goal):
        pass

#end of class method

#load grid
grid np.load('cspace.npy')
start = np.array([100.0, 100.0])
goal = np.array([700.0, 250.0])
numIterations = 200
stepSize = 50
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

fig = plt.figure("RRT Algorith")
plt.


