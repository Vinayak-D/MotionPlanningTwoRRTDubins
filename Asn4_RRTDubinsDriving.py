#RRT algorithm
import numpy as np
import matplotlib.pyplot as plt
import random

from Asn1_DubinsPath import atan2, norm, r2d, pi, cos, sin, returnDubinsPath

#TODO corresponds to student section, you have to complete this
#DONE------------ corresponds to sections already completed, you do not have to change anything

#DONE------------ tree Node class
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
class RRTDrivingAlgorithm():
    def __init__(self, start, goal, numIterations, grid, turnRadius, psiTarget):
        self.randomTree = treeNode(start[0], start[1], 0)           #The RRT (root position) and heading 
        self.goal = treeNode(goal[0], goal[1], psiTarget)           #goal position and heading
        self.turnRadius = turnRadius                                #Dubins path turn radius
        self.nearestNode = None                                     #nearest node            
        self.iterations = min(numIterations, 200)                   #number of iterations to run
        self.grid = grid                                            #the map
        self.rho = 5*self.turnRadius                                #length of each branch 
        self.path_distance = 0                                      #total path distance  
        self.nearestDist = 10000                                    #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                       #number of waypoints
        self.Waypoints = []                                         #the waypoints
        self.xPathTrajectory = []                                   #x trajectory of entire path
        self.yPathTrajectory = []                                   #y trajectory of entire path  
        self.goalCosts = [5000]                                     #cost to goal
        
    #TODO: add the node to the nearest node, and add goal if necessary    
    def addChild(self, locationX, locationY, heading, distanceToParent, xTP, yTP):
        if (locationX == self.goal.locationX):
            #TODO: add goal to the nearest node's children along with setting goal's parent, parentDistance, xTP, yTP, delete "pass"
            pass
        else:    
            tempNode = treeNode(locationX, locationY, heading)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode
            tempNode.parentDistance = distanceToParent
            tempNode.xTP = xTP
            tempNode.yTP = yTP
        
    #DONE------------ sample random point within search area limits
    def sampleNewPoint(self, x, y, psi):
        r0 = 200*np.random.random()
        seq = [-1,1]
        s = random.sample(seq, 1)
        tht = psi + s[0]*np.random.random()*pi/2
        xSample = x + r0*cos(tht)
        ySample = y + r0*sin(tht)
        return np.array([xSample, ySample])
    
    #TODO: steer a distance stepSize from start location to end location (keep in mind the grid limits)
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
        u_hat = v/norm(v)
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
        for child in root.children:
            self.findNearest(child, point)

    #DONE------------ find euclidean distance between a node and an XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)         
        return dist
    
    #DONE------------ check if the goal is reached within step size
    def goalFound(self,point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        return False
    
    #DONE------------ reset nearestNode and nearestDistance
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000
        
    #TODO: find unique path length from root of a node (cost)
    def findPathDistance(self, node):
        costFromRoot = 0
        currentNode = node
        #TODO: while currentNode location X is not the tree location X
        while False: #update this
            #TODO: update costFromRoot
            #TODO: set currentNode to it's parent
            #TODO: break if detect cycle (currentNode location X is node location X and currentNode location Y is node location Y)
            if True: #update this
                print('cycle detected!')
                break;
        return costFromRoot         
    
    #trace the path from goal to start, since have to reset if called many times, do this iteratively
    def retracePath(self):
        #TODO: reset path_distance, numWaypoints, Waypoints, xPathTrajectory and yPathTrajectory to their default values
        
        goal = self.goal
        
        #TODO: while goal location X is not the tree location X
        while False:
            #TODO: add 1 to numWaypoints
            currentPoint = np.array([goal.locationX, goal.locationY])
            self.Waypoints.insert(0,currentPoint)
            #TODO: add each element of xTP and yTP to xPathTrajectory and yPathTrajectory in reverse, from the beginning (insert)
            #TODO: increment path_distance by the parentDistance
            
            #TODO: assign goal to it's parent
        self.goalCosts.append(self.path_distance)  
                
#end of class definitions
#---------------------------------------------------------------------------------------------------------#
