import numpy as np
import matplotlib.pyplot as plt
import random
from Asn2_RRTDubins import RRTAlgorithm
from Asn1_DubinsPath import atan2, r2d, returnDubinsPath

#TODO corresponds to student section, you have to complete this
#DONE------------ corresponds to sections already completed, you do not have to change anything
        
#DONE------------ load the grid, set start and goal <x, y> positions, number of iterations, step size
grid = np.load('cspace.npy')
start = np.array([100.0, 100.0])
goal = np.array([1600.0, 600.0])
numIterations = 250
turnRadius = 25
goalRegion = plt.Circle((goal[0], goal[1]), 3*turnRadius, color='b', fill = False)

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.plot(start[0],start[1],'ro')
plt.plot(goal[0],goal[1],'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')
 

#TODO: Complete the RRT algorithm (after you are done with Assignment 2.1: the class definitions)

#Begin
rrt = RRTAlgorithm(start, goal, numIterations, grid, turnRadius)
#iterate
plt.pause(2)

for i in range(rrt.iterations):
    
    #TODO: Reset nearest values

    #algorithm begins here
    
    #TODO: call the rrt.sample a point method and assign to point
    point = 0 #update
    
    #TODO: find nearest node (call the method)
    
    #steer from the nearest node to point, and assign to "new"
    new = 0 #update
    
    #TODO: find heading to the goal from this "new" point and add a random value to it (update "0")
    heading = 0 + random.random()
    print("Iteration: ",i, "Heading: ", r2d(heading))
    
    #TODO: if not in obstacle, return dubins path from nearest node to new
    if not rrt.isInObstacle(rrt.nearestNode, new):
        
        #TODO: return Dubins path from nearest node to new point (call returnDubinsPath and pass in the appropriate 7 arguments)
        xTP, yTP, pathDistance = 0, 0, 0 #update this
        
        #TODO: if pathDistance is not None and xTP, yTP (the Dubins path) does not fall in an obstacle, add "new" as a child and plot
        if True: #update this
            #addChild(......) - update this too
            plt.pause(0.05)
            plt.plot(xTP,yTP,'k')
            plt.plot(new[0], new[1], marker="x", markersize=3, markeredgecolor="k", markerfacecolor="k")
            
        #TODO: if goal found, check if dubins path exists to nearest node and return
        if True: #update this
            #TODO: find Dubins path from nearest node to goal (the heading of the goal can be 0)
            xTP, yTP, pathDistance = 0, 0, 0
            
            #TODO: if pathDistance is not None and xTP, yTP (the Dubins path) does not fall in an obstacle, add "goal" as a child and plot
            if True: #update this
                #addChild(......) - update this too
                plt.plot(xTP,yTP,'k')
                
                #TODO: retrace path from goal to start
                
                #DONE------------ display route data and exit
                rrt.Waypoints.insert(0,start)
                print("Goal found!")
                print("Number of waypoints: ", rrt.numWaypoints)
                print("Path Distance (m): ", rrt.path_distance) 
                plt.plot(rrt.xPathTrajectory, rrt.yPathTrajectory, 'b')
                for i in range(rrt.numWaypoints - 2):
                    plt.plot(rrt.Waypoints[i+1][0], rrt.Waypoints[i+1][1], marker="x", markersize=5, markeredgecolor="b", markerfacecolor="b")
                    text = str(round(rrt.Waypoints[i+1][0])) + ', ' + str(round(rrt.Waypoints[i+1][1]))
                    plt.text(rrt.Waypoints[i+1][0] + 10, rrt.Waypoints[i+1][1] + 10, text, fontsize=6, fontweight="bold",color="r")
                break
            else:
                print('no path found')

#DONE------------ plot the final trajectory and waypoints
plt.figure("Trajectory")
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.plot(rrt.xPathTrajectory, rrt.yPathTrajectory, 'b')
for i in range(rrt.numWaypoints - 2):
    plt.plot(rrt.Waypoints[i+1][0], rrt.Waypoints[i+1][1], marker="x", markersize=5, markeredgecolor="b", markerfacecolor="b")
    text = str(round(rrt.Waypoints[i+1][0])) + ', ' + str(round(rrt.Waypoints[i+1][1]))
    plt.text(rrt.Waypoints[i+1][0] + 10, rrt.Waypoints[i+1][1] + 10, text, fontsize=7, fontweight="bold",color="r")            
plt.xlim([0,grid.shape[1]])
plt.ylim([0,grid.shape[0]])    
    