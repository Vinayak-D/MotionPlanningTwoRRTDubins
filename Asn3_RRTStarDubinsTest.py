import numpy as np
import matplotlib.pyplot as plt
import random
from Asn3_RRTStarDubins import RRTStarAlgorithm, treeNode
from Asn1_DubinsPath import atan2, r2d, returnDubinsPath

#TODO corresponds to student section, you have to complete this
#DONE------------ corresponds to sections already completed, you do not have to change anything

#DONE------------ load the grid, set start and goal <x, y> positions, number of iterations, step size
grid = np.load('cspace.npy')
start = np.array([300.0, 300.0])
goal = np.array([1400.0, 775.0])
numIterations = 300
turnRadius = 25
goalRegion = plt.Circle((goal[0], goal[1]), 3*turnRadius, color='b', fill = False)

fig = plt.figure("RRT Star Algorithm")
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.plot(start[0],start[1],'ro')
plt.plot(goal[0],goal[1],'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

#TODO: Complete the RRTStar algorithm (after you are done with the class definitions)
    
#Begin
rrtStar = RRTStarAlgorithm(start, goal, numIterations, grid, turnRadius)
#iterate
plt.pause(2)

for i in range(rrtStar.iterations):
    
    #TODO: Reset nearest values
    
    #algorithm begins here
    
    #TODO: sample a point and assign to "point"
    
    #TODO: find nearest node
    
    #steer from the nearest node to point, and assign to "new"
    new = 0
    
    #TODO: find heading to the goal from this new point and add a random value to it (update "0" in line 48)
    heading = 0
    
    print("Iteration: ",i, "Heading: ", r2d(heading))
    #TODO: if not in obstacle, return dubins path from nearest node to new
    if not rrtStar.isInObstacle(rrtStar.nearestNode, new):
        
        #TODO: return Dubins path from nearest node to new point
        xTP, yTP, pathDistance = 0,0,0
        
        #TODO: if pathDistance is not None and xTP, yTP (the Dubins path) does not fall in an obstacle, find the minimum cost node
        
        if True: #update this
        
            #TODO: find neighbouring nodes
            
            #TODO: assign "min_cost_node" to nearest node, 
            
            #TODO: assign "min_cost_xTrajectory" to xTP and "min_cost_yTrajectory "to yTP

            #TODO: assign "min_cost_distance" to pathDistance
            
            #TODO: assign "min_cost" to the cost of the min_cost_node (use appropriate method)
            
            #add pathDistance to min_cost, remove "pass"
            pass
            
        else:
            continue
        
        #TODO: connect along minimum cost path
        for vertex in rrtStar.neighbouringNodes:
            vertex_cost = rrtStar.findPathDistance(vertex)
            
            #TODO: get dubins path between vertex (a treeNode) object, and new
            xTP, yTP, pathDistance = 0, 0, 0
            
            #TODO: if pathDistance is not None and xTP, yTP (the Dubins path) does not fall in an obstacle,
            if True: #update this
            
                #add path distance to vertex cost
                
                #if vertex cost is less than the min_cost (from line 71)
                if True: #update this
                    
                    #TODO: update "min_cost_node" to vertex, 
                    
                    #TODO: update "min_cost_xTrajectory", "min_cost_yTrajectory", "min_cost_distance" to the new values in line 80
                    
                    #TODO: update "min_cost" to vertex cost, remove "pass"
                    
                    pass
          
                
        #DONE------------  update nearest node to the min cost node, create a treeNode object, and add child, then plot
        
        #update nearest node, and add to new node (if it clears obstacle), 
        #otherwise it'll add to the original nearest node (obstacle free)     
        rrtStar.nearestNode = min_cost_node  
        #update trajectory again from the updated nearest (min cost) node            
        newNode = treeNode(new[0], new[1], heading)
        newNode.parentDistance = min_cost_distance
        newNode.xTP = min_cost_xTrajectory
        newNode.yTP = min_cost_yTrajectory
        rrtStar.addChild(newNode, rrtStar.nearestNode)   
        #plot for display
        plt.pause(0.05)
        plt.plot(min_cost_xTrajectory, min_cost_yTrajectory, 'k')
        plt.plot(new[0], new[1], marker="x", markersize=3, markeredgecolor="k", markerfacecolor="k")
        
        #TODO: rewire tree    
        for vertex in rrtStar.neighbouringNodes:
            vertex_cost = min_cost
            if vertex.locationX != newNode.locationX and vertex.locationY != newNode.locationY:
                
                #TODO: return Dubins Path between vertex and new[0], new[1], heading
                
                xTP, yTP, pathDistance = 0, 0, 0
                
                #TODO: if pathDistance is not None and xTP, yTP (the Dubins path) does not fall in an obstacle,
                if True: #update this
                    #TOD0: add path distance to vertex_cost (line 119)
                    
                    #TOD0: if vertex_cost is less than the path distance from the vertex to the root
                    if True: #update this
                        #TODO: update the vertex.parent to newNode, 
                        #TODO: update vertex.xTP, vertex.yTP, vertex.parentDistance accordingly

                        rrtStar.rewireCount += 1
                        
        #TODO: if goal found, append to path, trigger flag, let it sample more
        point = np.array([newNode.locationX, newNode.locationY])
        if rrtStar.goalFound(point):
            
            xTP, yTP, pathDistance = returnDubinsPath(newNode.locationX, newNode.locationY, heading, goal[0], goal[1], 
                                                      0, rrtStar.turnRadius)
            
            #TODO: if path distance is not None
            if True: #update this
            
                #find projected cost, which is the distance from newNode to root + pathDistance
                projectedCost = 0
                
                #TODO: if projected cost is less than the last value of rrtStar.goalCosts
                if True: #update this
                
                    
                    rrtStar.goal.parentDistance = pathDistance
                    rrtStar.goal.xTP = xTP
                    rrtStar.goal.yTP = yTP
                    rrtStar.addChild(rrtStar.goal, newNode)
                    plt.plot(xTP, yTP, 'k') 
                    #DONE------------ retrace and plot, this method finds waypoints and cost from root
                    rrtStar.retracePath()
                    print("Goal Cost: ", rrtStar.goalCosts)
                    plt.pause(0.5)
                    rrtStar.Waypoints.insert(0,start)
                    #DONE------------ plot the waypoints
                    for i in range(rrtStar.numWaypoints - 2):
                        plt.plot(rrtStar.Waypoints[i+1][0], rrtStar.Waypoints[i+1][1], marker="x", markersize=5, markeredgecolor="b", markerfacecolor="b")
                        plt.pause(0.01)
                    plt.plot(rrtStar.xPathTrajectory, rrtStar.yPathTrajectory, 'b')



#DONE------------ print final waypoints and path distance
for i in range(rrtStar.numWaypoints - 2): 
    text = str(round(rrtStar.Waypoints[i+1][0])) + ', ' + str(round(rrtStar.Waypoints[i+1][1]))
    plt.text(rrtStar.Waypoints[i+1][0] + 10, rrtStar.Waypoints[i+1][1] + 10, text, fontsize=6, fontweight="bold",color="r")
print("Number of waypoints: ", rrtStar.numWaypoints)
print("Minimum Path Distance (m): ", rrtStar.path_distance)     
plt.figure("Trajectory")
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.plot(rrtStar.xPathTrajectory, rrtStar.yPathTrajectory)
for i in range(rrtStar.numWaypoints - 2):
    plt.plot(rrtStar.Waypoints[i+1][0], rrtStar.Waypoints[i+1][1], marker="x", markersize=7, markeredgecolor="b", markerfacecolor="b")
    text = str(round(rrtStar.Waypoints[i+1][0])) + ', ' + str(round(rrtStar.Waypoints[i+1][1]))
    plt.text(rrtStar.Waypoints[i+1][0] + 10, rrtStar.Waypoints[i+1][1] + 10, text, fontsize=7, fontweight="bold",color="r")
plt.plot(rrtStar.xPathTrajectory, rrtStar.yPathTrajectory, 'b')