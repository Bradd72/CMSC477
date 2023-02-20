import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import time

# 0 = Empty
# 1 = Wall
# 2 = Start
# 3 = End
# 4 = Queued
# 5 = Visited
# 6 = Found Path
# 7 = Wall Boundary
# 8 = Robot
# [x, y, distance]

def Dijkstra(maze, s):
    startTime = time.time()
    queue = [s]         # Queue the start
    maze[s[0],s[1]] = 4 
    nodeDistances[s[0],s[1]] = s[2]

    queueflag = False;
    pathEdges = {}
    while queue != []:
        currNode = queue.pop(0) # FIFO queue
        currNode[2] = nodeDistances[currNode[0],currNode[1]]
        
        # Check surrounding 8 nodes
        for i in [[-1,-1,1.4142],[0,-1,1.0],[1,-1,1.4142],[-1,0,1.0],[1,0,1.0],[-1,1,1.4142],[0,1,1.0],[1,1,1.4142]]:   
            adjNode = [currNode[0]+i[0],currNode[1]+i[1],currNode[2]+i[2]]
            # end node: quit
            if maze[adjNode[0],adjNode[1]] == 3:    
                queueflag = True;
                if adjNode[2] < nodeDistances[adjNode[0],adjNode[1]]:
                    nodeDistances[adjNode[0],adjNode[1]] = adjNode[2]
                    pathEdges[tuple([adjNode[0],adjNode[1]])] = tuple([currNode[0],currNode[1]])
                    break  
            # open node: add to queue                           
            elif maze[adjNode[0],adjNode[1]] == 0:    
                # Weighting fastest path
                if adjNode[2] < nodeDistances[adjNode[0],adjNode[1]]:
                    nodeDistances[adjNode[0],adjNode[1]] = adjNode[2]
                    if queueflag == False:
                        pathEdges[tuple([adjNode[0],adjNode[1]])] = tuple([currNode[0],currNode[1]])
                if queue == [] and queueflag == False:
                    queue.append(adjNode)
                    maze[adjNode[0],adjNode[1]] = 4
                elif queueflag == False:
                    tn = 0
                    for entry in queue:
                        if adjNode[2] < entry[2]:
                            queue.insert(tn,adjNode)
                            maze[adjNode[0],adjNode[1]] = 4
                            break
                        elif adjNode[2] >= entry[-1]:
                            queue.append(adjNode)
                            maze[adjNode[0],adjNode[1]] = 4
                            break
                        tn += 1

            elif maze[adjNode[0],adjNode[1]] == 4:
                if adjNode[2] < nodeDistances[adjNode[0],adjNode[1]]:
                    nodeDistances[adjNode[0],adjNode[1]] = adjNode[2]
                    if queueflag == False:
                        pathEdges[tuple([adjNode[0],adjNode[1]])] = tuple([currNode[0],currNode[1]])

        maze[currNode[0],currNode[1]] = 5   # mark current as Visitied                

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
        ycoords.append(height-node[0])
        xcoords.append(node[1])

    plt.plot(xcoords,ycoords,'r-')
    return

def ExpandWalls(maze):
    '''
    Creates a 1 unit wide boundary around the known walls
    '''
    for h in range(height-1):
        for w in range(width-1):
            if maze[h,w] == 1:
                if h == 0 or w == 0:
                    continue
                for i in [[-1,-1,1.4142],[0,-1,1.0],[1,-1,1.4142],[-1,0,1.0],[1,0,1.0],[-1,1,1.4142],[0,1,1.0],[1,1,1.4142]]:
                    if maze[h+i[0],w+i[1]] == 0:
                        maze[h+i[0],w+i[1]] = 7
    return

def Draw_Maze(mazelist):
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
                Wally.append(height-rowCounter)
            elif entry == 7:    # Wall Boundary
                Boundx.append(entryCounter)
                Boundy.append(height-rowCounter)
            elif entry == 2:    # Start
                ax.plot(entryCounter, height-rowCounter, 'bo')
            elif entry == 3:    # End
                ax.plot(entryCounter, height-rowCounter, 'go')
            entryCounter += 1
        rowCounter += 1
        entryCounter = 0
    plt.scatter(Wallx,Wally,c='k',marker='s')
    plt.scatter(Boundx,Boundy,c='#9c9c9c',marker='s')

    ax.axis('equal')
    ax.grid()
    ax.set_xticks(range(0, width))
    ax.set_xticklabels(range(1, width+1))
    ax.set_yticks(range(0, height))
    ax.set_yticklabels(range(1, height+1))
    calcTime = time.time()-startTime
    print('Plot: {} seconds'.format(calcTime))
    plt.pause(1e-10)
    return

if __name__ == '__main__':
    # FOR INITIAL LAB WORK, KNOWN ENVIRONMENT
    '''
    - init maze
    - determine current location in maze
        - Apriltag distances give relative location
        - Apriltag id gives location in map
    - solve path
    - follow path
        - FIND A WAY TO DETERMINE NEXT POINTS/PATH
        - interpolate between next point to find desired location given a certain time or velocity between
        - PID controller to move robot to point moving in space (in time!)
            - Use other apriltags to determine position in maze and calculate error offset
    '''
    # Init maze
    mazeList = pd.read_csv("Lab_1\Lab1Map.csv", header=None).to_numpy()
    height, width = mazeList.shape
    nodeDistances = float(16384)*np.ones(mazeList.shape, dtype=int)
    start = np.where(mazeList==2)
    startLoc = np.array([start[0][0],start[1][0]])
    # Start interactive plot
    plt.ion()
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111)
    ExpandWalls(mazeList)
    Draw_Maze(mazeList)

    # Determine current location in maze
    robotLoc = startLoc
    
    # Solve Path
    shortestPath = Dijkstra(mazeList, [robotLoc[0],robotLoc[1],0])
    print(shortestPath)
    PlotPath(shortestPath)

    # Follow Path


    # Keeping the updating plot open
    while True:
        plt.pause(10)


    # FOR BONUS, UNKNOWN ENVIRONMENT
    '''
    - init BLANK maze with start and end
    - look for apriltag to get current robot position in maze
        - if one exists mark that location as a wall in the maze and get relative position
        - if one does not exist, assume at start
        - solve a-star from current location to get path
    - follow path until new wall is found
        - update map with wall and resolve a-star
    '''