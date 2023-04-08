#Assignment 0
#Review of Python

#Imports and Definitions
import numpy as np
import matplotlib.pyplot as plt
cos = np.cos
sin = np.sin
atan2 = np.arctan2

#Random points
x1 = 500
y1 = 200
psi1 = 0.45
x2 = 1000
y2 = 430
psi2 = -1.35

length = 40
width = 15

#Plotting
fig = plt.figure("Configuration Space")
grid = np.load('cspaceRoadmap.npy')
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.arrow(x1, y1, length * cos(psi1), length * sin(psi1), fc="b", ec="b", head_width=width, head_length=width)
#TODO: Plot arrow for x2, y2 and psi2 (similar to above), in red color ("r")
    
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

#Determine angle psi3 between <x1,x2> and <y1,y2> (atan2(dy,dx)), and plot the arrow on <x1,y1>
psi3 = 0 #TODO: update this
plt.plot(0,0) #TODO: update this 

#Configuration Space - for a point use grid[y][x]
print(grid[754][345])
print(grid[345][229])
#TODO: print grid at x = 1455, y = 765
print(grid.shape)
print("--------------------------")

#Trees
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.children = []                        #children list   
        self.parent = None                        #parent 
        
#Tree Traversal - Recursive DFS
def DFS(node):
    print(node.locationX, node.locationY)
    for child in node.children:
        DFS(child)

A = treeNode(45, 67)
B = treeNode(100,100)
A.children.append(B)
B.parent = A

#TODO: Create treeNode C at <145,254> and connect it to B, 
#TODO: Create treeNode D at <65,65> and connect it to C

print("Depth First Search")
DFS(A)
