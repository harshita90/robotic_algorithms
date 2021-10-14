#!/usr/bin/env python
import numpy as np
import math


class Node:
    def __init__(self,point):
        self.point = point
        self.parent = None
        self.G = 0
        self.H = 0
        self.F = 0

def allnodes():
    nods = np.empty([20, 18], dtype = Node)
    for i in range(0,20):
        for j in range(0,18):
            nods[i][j] = Node((i,j))
    return nods

def nodes(current,mp,nods):
    node = []
    for i in range(-1,2):
        for j in range(-1,2):
            y,x = current.point
            y = round(y+i)
            x = round(x+j)
            n = (y,x)
            a,b = mp.shape
            if (i*j == 0 and i!=j):
                if ((mp[y,x]==0) and (y in range(a-1)) and (x in range(b-1))):
                    node.append(nods[y][x])
    return node


def astar(mp,start_map,goal_map,nods):
    current = Node(start_map)
    end = Node(goal_map)
    path = []
    visited = []
    neighbour = [current]
    atGoal = False
    while (atGoal == False):
        best = min(neighbour,key=lambda nod:nod.F)
        current = best
        neighbour.remove(current)
        visited.append(current)
        #Check if goal is reached then return the A* path
        if (current.point[0] == end.point[0]) and (current.point[1] == end.point[1]):
            atGoal = True
            print("Goal reached")
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            return path
        #If path is not reached, then iterate through the neighbours of current node
        node = nodes(current,mp,nods)
        for no in node:
            if no in visited:
                continue
            if no not in neighbour:
                l,m = current.point
                no.G = current.G + 1
                no.H = math.sqrt((goal_map[0] - l)**2 + (goal_map[1] - m)**2)
                no.F = no.G + no.H
                no.parent = current
                neighbour.append(no)
            else:
                nG = current.G + 1
                if nG<current.G:
                    no.G = nG
                    no.F = no.G + no.H
                    no.parent = current
    raise ValueError("No possible path")

if __name__ == '__main__':
 s = (-8.0,-2.0)
 g = (5.0,9.0)
 map = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]
 map = np.asarray(map).reshape((20,18))
 start_map = (round(-s[1]+9),round(s[0]+8))
 goal_map = (round(-g[1]+9),round(g[0]+8))
 print(start_map)
 print(goal_map)
 nods = allnodes()
 path = astar(map,start_map,goal_map,nods)
 for k in range(len(path)):
    y,x = path[k].point
    map[y,x] = 8
 print("Best path shown by digit 8")
 print(map)
 
