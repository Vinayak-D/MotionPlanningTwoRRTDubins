#RRT algorithm
import numpy as np
import matplotlib.pyplot as plt
import random

from Asn1_DubinsPath import sqrt

#TODO corresponds to student section, you have to complete this
#DONE------------ corresponds to sections already completed, you do not have to change anything

#DONE------------ treeNode class
class treeNode():
    def __init__(self, locationX, locationY, heading):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.heading = heading                    #Heading (rad)
        self.children = []                        #children list   
        self.parent = None                        #parent node reference 
        self.parentDistance = 0                   #distance to parent node
        self.xTP = None                           #x values of dubins path to parent
        self.yTP = None                           #y values of dubins path to parent
        
#RRT Algorithm class
class RRTStarAlgorithm():
    def __init__(self, start, goal, numIterations, grid, turnRadius):
        self.randomTree = treeNode(start[0], start[1], 0)          #The RRT (root position) (has 0 cost)
        self.goal = treeNode(goal[0], goal[1], 0)                  #goal position (initialize to a high cost)
        self.turnRadius = turnRadius                               #Dubins path turn radius
        self.nearestNode = None                                    #nearest node            
        self.iterations = min(numIterations, 400)                  #number of iterations to run
        self.grid = grid                                           #the map
        self.rho = 5*self.turnRadius                               #length of each branch   
        self.nearestDist = 10000                                   #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                      #number of waypoints
        self.path_distance = 0                                     #total path distance  
        self.Waypoints = []                                        #the waypoints
        self.searchRadius = self.rho*2                             #the radius to search for finding neighbouring vertices 
        self.neighbouringNodes = []                                #neighbouring nodes  
        self.goalCosts = [10000]                                   #the costs to the goal (ignore first value)
        self.rewireCount = 0                                       #number of times the tree was rewired
        self.xPathTrajectory = []                                  #x trajectory of entire path
        self.yPathTrajectory = []                                  #y trajectory of entire path        
            
    #DONE------------ add the node to the nearest node, and add goal if necessary    
    def addChild(self, treeNode, nodeParent):
        if (treeNode.locationX == self.goal.locationX):
            nodeParent.children.append(self.goal)
            self.goal.parent = nodeParent
        else:    
            self.nearestNode.children.append(treeNode)
            treeNode.parent = self.nearestNode
        
    #DONE------------ sample random point within grid limits
    def sampleAPoint(self):
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
        point = np.array([x, y])
        return point
    
    #TODO steer a distance stepSize from start location to end location (keep in mind the grid limits)
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        #TODO: constrain above point in x and y respectively (remember x, y is grid[y,x]), check min and max limits


        
        return point
    
    #DONE------------ check if obstacle lies between the start and end point of the edge
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = min(self.grid.shape[1]-1,locationStart.locationX + i*u_hat[0])
            testPoint[1] = min(self.grid.shape[0]-1,locationStart.locationY + i*u_hat[1])
            if self.grid[round(testPoint[1]),round(testPoint[0])] == 1:
                return True
        return False    
    
    #TODO: check if the Dubins trajectory falls in an obstacle
    def isDubinsInObstacle(self, xTrajectory, yTrajectory):
        #TODO: iterate through trajectory and if any point lies in an obstacle, return True
        #be sure to check for grid limits, again remember x,y is grid[y,x]
        
        return False  

    #DONE------------ find the unit vector between 2 locations
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    #DONE------------ find the nearest node from a given (unconnected) point (Euclidean distance)
    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        #update nearest node
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        #recursive call
        for child in root.children:
            self.findNearest(child, point)
    
    #DONE------------ find neighbouring nodes within the search radius
    def findNeighbouringNodes(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        #add to neighbouringNodes if within radius
        if dist <= self.searchRadius and dist > 10:
            self.neighbouringNodes.append(root)
        #recursive call
        for child in root.children:
            self.findNeighbouringNodes(child, point)        

    #DONE------------ find euclidean distance between a node and an XY point
    def distance(self, node1, point):
        dist = sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)         
        return dist
    
    #DONE------------ check if the goal is reached within step size
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        return False
    
    #DONE------------ reset nearestNode, nearestDistance, neighbouring Nodes
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000
        self.neighbouringNodes = []
    
    #TODO: trace the path from goal to start, since have to reset if called many times, do this iteratively
    def retracePath(self):
        #TODO: reset path_distance, numWaypoints, Waypoints, xPathTrajectory and yPathTrajectory to their default values


        goal = self.goal
        
        #TODO: while goal location X is not the tree location X
        while False: #update this
            #TODO: add 1 to numWaypoints
            currentPoint = np.array([goal.locationX, goal.locationY])
            self.Waypoints.insert(0,currentPoint)
            #TODO: add each element of goal.xTP and ygoal.yTP to xPathTrajectory and yPathTrajectory in reverse, from the beginning (insert)
            #TODO: increment path_distance by the goal.parentDistance
            
            #TODO: assign goal to it's parent
            
        self.goalCosts.append(self.path_distance)    
        
    #TODO: find unique path length from root of a node (cost)
    def findPathDistance(self, node):
        costFromRoot = 0
        currentNode = node
        #TODO: while currentNode location X is not the tree location X
        while False: #update this
            #TODO: update costFromRoot (line 157)
            #TODO: set currentNode to it's parent
            #TODO: break if detect cycle (currentNode location X is node location X and currentNode location Y is node location Y)
            if True: #update this
                print('cycle detected!')
                break;
        return costFromRoot    
        
#end of class definitions
#------------------------------------------------------------------------------#

