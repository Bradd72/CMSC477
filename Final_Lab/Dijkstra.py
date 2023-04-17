import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import time
import random

random.seed('Final Lab',version=2)

# 0 = Empty
# 1 = Wall
# 2 = Start
# 3 = End
# 4 = Queued
# 5 = Visited
# 6 = Found Path
# 7/70 = Wall Boundary
# 8 = Robot
# [x, y, distance]

def Dijkstra(maze, s):
    nodeDistances = float(16384)*np.ones(maze.shape, dtype=int)

    startTime = time.time()
    queue = [s]         # Queue the start
    maze[s[1],s[0]] = 4 
    nodeDistances[s[1],s[0]] = s[2]

    queueflag = False
    pathEdges = {}
    while queue != []:
        currNode = queue.pop(0) # FIFO queue
        currNode[2] = nodeDistances[currNode[1],currNode[0]]
        
        # Check surrounding 8 nodes
        for i in [[-1,-1,1.4142],[0,-1,1.0],[1,-1,1.4142],[-1,0,1.0],[1,0,1.0],[-1,1,1.4142],[0,1,1.0],[1,1,1.4142]]:   
            adjNode = [currNode[0]+i[0],currNode[1]+i[1],currNode[2]+i[2]]
            # end node: quit
            if maze[adjNode[1],adjNode[0]] == 3:    
                queueflag = True
                if adjNode[2] < nodeDistances[adjNode[1],adjNode[0]]:
                    nodeDistances[adjNode[1],adjNode[0]] = adjNode[2]
                    pathEdges[tuple([adjNode[0],adjNode[1]])] = tuple([currNode[0],currNode[1]])
                    break  
            # open node: add to queue                           
            elif maze[adjNode[1],adjNode[0]] == 0:    
                # Weighting fastest path
                if adjNode[2] < nodeDistances[adjNode[1],adjNode[0]]:
                    nodeDistances[adjNode[1],adjNode[0]] = adjNode[2]
                    if queueflag == False:
                        pathEdges[tuple([adjNode[0],adjNode[1]])] = tuple([currNode[0],currNode[1]])
                if queue == [] and queueflag == False:
                    queue.append(adjNode)
                    maze[adjNode[1],adjNode[0]] = 4
                elif queueflag == False:
                    tn = 0
                    for entry in queue:
                        if adjNode[2] < entry[2]:
                            queue.insert(tn,adjNode)
                            maze[adjNode[1],adjNode[0]] = 4
                            break
                        elif adjNode[2] >= entry[-1]:
                            queue.append(adjNode)
                            maze[adjNode[1],adjNode[0]] = 4
                            break
                        tn += 1

            elif maze[adjNode[1],adjNode[0]] == 4:
                if adjNode[2] < nodeDistances[adjNode[1],adjNode[0]]:
                    nodeDistances[adjNode[1],adjNode[0]] = adjNode[2]
                    if queueflag == False:
                        pathEdges[tuple([adjNode[0],adjNode[1]])] = tuple([currNode[0],currNode[1]])

        maze[currNode[1],currNode[0]] = 5   # mark current as Visitied                

    shortestPath = [tuple(list(pathEdges)[-1])]
    while True:
        if shortestPath[0] in pathEdges:
            shortestPath.insert(0,pathEdges[shortestPath[0]])
        else:
            break

    calcTime = time.time()-startTime
    print('Search: {} seconds'.format(calcTime))

    return shortestPath

def PlotPath(path):
    xcoords = []
    ycoords = []
    for node in path:
        ycoords.append(-node[1])
        xcoords.append(node[0])

    plt.plot(xcoords,ycoords,'r:')
    plt.pause(1e-10)
    return

def ExpandWalls(maze,padding):
    '''
    Creates a 1 unit wide boundary around all known walls
    '''
    height, width = maze.shape
    for i in range(padding):
        for h in range(height-1):
            for w in range(width-1):
                if maze[h,w] == 1 or maze[h,w] == 7: # Wall
                    if h == 0 or w == 0:
                        continue
                    for i in [[-1,-1,1.4142],[0,-1,1.0],[1,-1,1.4142],[-1,0,1.0],[1,0,1.0],[-1,1,1.4142],[0,1,1.0],[1,1,1.4142]]:
                        if maze[h+i[0],w+i[1]] == 0:
                            maze[h+i[0],w+i[1]] = 70
        
        for h in range(height-1):
            for w in range(width-1):
                if maze[h,w] == 70: # boundary hold
                    maze[h,w] = 7
    return

def Draw_Maze(mazelist, ax):
    startTime = time.time()
    # Loop over all points in maze
    Wallx = []
    Wally = []
    Boundx = []
    Boundy = []
    rowCounter = 0
    entryCounter = 0
    for row in mazelist:
        for entry in row:       
            # Plotting maze outline and POI
            if entry == 1:      # Obstacle
                Wallx.append(entryCounter)
                Wally.append(-rowCounter)
            elif entry == 7:    # Wall Boundary
                Boundx.append(entryCounter)
                Boundy.append(-rowCounter)
            elif entry == 2:    # Start
                ax.plot(entryCounter, -rowCounter, 'bo')
            elif entry == 3:    # End
                ax.plot(entryCounter, -rowCounter, 'go')
            entryCounter += 1
        rowCounter += 1
        entryCounter = 0
    plt.scatter(Wallx,Wally,c='k',marker='s')
    plt.scatter(Boundx,Boundy,c='#9c9c9c',marker='s')

    ax.axis('equal')
    calcTime = time.time()-startTime
    print('Plot: {} seconds'.format(calcTime))
    plt.pause(1e-10)
    return


if __name__ == '__main__':
    # Init maze
    mazeList = pd.read_csv("Labs\Final_Lab\Lab1Map.csv", header=None).to_numpy()
    # height, width = mazeList.shape
    start = np.where(mazeList==2)
    startLoc = np.array([start[1][0],start[0][0]])
    # Start interactive plot
    plt.ion()
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111)
    ExpandWalls(mazeList,padding=4)
    Draw_Maze(mazeList,ax=ax)

    # Determine current location in maze
    robotLoc = startLoc
    
    # Solve Path
    shortestPath = Dijkstra(mazeList, [robotLoc[0],robotLoc[1],0])
    PlotPath(shortestPath)

    # Keeping the updating plot open
    while True:
        plt.pause(10)