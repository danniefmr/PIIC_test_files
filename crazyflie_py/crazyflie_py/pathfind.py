from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt
import random

from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 22

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
        self.goal = treeNode(goal[0], goal[1])
        self.nearestNode = None
        self.iterations = min(numIterations, 200)
        self.grid = grid
        self.rho = stepSize
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints= []

    #add node to nearest node and add goal when reached
    def addChild(self, locationX, locationY):
        if(locationX == self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(locationX,locationY)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode

    #sample a random point within arena limits
    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
                    
        point = np.array([x, y])
        print("problematic random point: ", point)
        return point

    #steer a distance stepsize from start to end of location
    def steerToPoint(self, locationStart, locationEnd):
        #print("loaction_End_inSteer to point: ", locationEnd)
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1]-1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0]-1
        return point

    #check if obstacle lies between the start and end point of the edge
    def isInObstacle(self, locationStart, locationEnd):
        #print("End locatation: ", locationEnd)
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = locationStart.locationX + i*u_hat[0]
            testPoint[1] = locationStart.locationX + i*u_hat[1]
            #print("problem value: ",round(testPoint[1]))
            #print("unraounded value", testPoint[1])
            #if round(testPoint[1]) >= 648:
             #   print("asjdahwdahwd")
            #else:
            if self.grid[round(testPoint[1]),round(testPoint[0])] == 1:
                    return True
        return False

    #find unit vector between 2 points which form a vector
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat

    #find nearest node from an unconnected point
    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)
        pass

    #find euclidean distance between a node and XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return dist
    
    #check if goal has been reached
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        pass

    #reset nearestNode and nearest Distance
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    #trace the path from goal to start
    def retraceRRTPath(self, goal):
        if goal.locationX == self.randomTree.locationX:
            return True
        self.numWaypoints += 1
        #insert currentPoint to the Waypoints array from the beginning
        currentPoint = np.array([self.goal.locationX, self.goal.locationY])
        self.Waypoints.insert(0,currentPoint)
        self.path_distance += self.rho
        self.retraceRRTPath(self.goal.parent)

#end of class method

#load grid
grid = np.load('cspace.npy')
start = np.array([100.0, 100.0])
goal = np.array([700.0, 250.0])
numIterations = 200
stepSize = 40
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

fig = plt.figure("RRT Algorith")
plt.imshow(grid, cmap='binary')
plt.plot(start[0],start[1],'ro')
plt.plot(goal[0],goal[1],'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

#Begin
rrt = RRTAlgorithm(start, goal, numIterations, grid, stepSize)

for i in range(rrt.iterations):
    rrt.resetNearestValues()
    print("Iteration: ",i)
    point = rrt.sampleAPoint()
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    print("new_point: ", new)
    bool = rrt.isInObstacle(rrt.nearestNode, new)
    if(bool == False):
        rrt.addChild(new[0], new[1])
        plt.pause(0.10)
        plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go', linestyle = "--")

        if(rrt.goalFound(new)):
            rrt.addChild(goal[0], goal[1])
            print("Goal found!")
            break

#trace bacl the path returned, and add start to waypoints
rrt.retraceRRTPath(rrt.goal)
rrt.Waypoints.insert(0,start)
print("Number of waypoints: ", rrt.numWaypoints)
print("Path Distance (m): ", rrt.path_distance)
print("Waypoints: ", rrt.Waypoints)

#plot waypoints
#plot the waypoints
for i in range(len(rrt.Waypoints)-1):
    plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]], 'ro', linestyle = "--")
    plt.pause(0.10)

plt.show()
'''
img = Image.open('cspace.png')

img = ImageOps.grayscale(img)

np_img = np.array(img)
np_img = ~np_img
np_img[np_img > 0] = 1
plt.set_cmap('binary')
plt.imshow(np_img)

np.save('cspace.npy', np_img)

grid = np.load('cspace.npy')
plt.imshow(grid)
plt.tight_layout()
plt.show()
'''
