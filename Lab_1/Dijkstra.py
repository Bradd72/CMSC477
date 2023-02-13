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
# [x, y, distance]

def Dijkstra(maze, s):
    startTime = time.time()
    queue = [s]         # Queue the start
    maze[s[0],s[1]] = 4 
    nodeDistances[s[0],s[1]] = s[2]

    depthCounter = 0    # number of nodes checked

    queueflag = False;

    modifiedNodes = []  # Used in plotting
    modifiedNodeStatus = []
    pathEdges = {}
    while queue != []:
        depthCounter += 1
        currNode = queue.pop(0) # FIFO queue
        currNode[2] = nodeDistances[currNode[0],currNode[1]]
        
        # Check surrounding 8 (or 4) nodes
        for i in [[-1,-1,1.4142],[0,-1,1.0],[1,-1,1.4142],[-1,0,1.0],[1,0,1.0],[-1,1,1.4142],[0,1,1.0],[1,1,1.4142]]:  
        #for i in [[0,-1,1],[-1,0,1],[1,0,1],[0,1,1]]:  
            adjNode = [currNode[0]+i[0],currNode[1]+i[1],currNode[2]+i[2]]
            print('[5,2] dist: {}'.format(nodeDistances[5,2]))
            # end node: quit
            if maze[adjNode[0],adjNode[1]] == 3:    
                queueflag = True;
                if adjNode[2] < nodeDistances[adjNode[0],adjNode[1]]:
                    nodeDistances[adjNode[0],adjNode[1]] = adjNode[2]
                    pathEdges[tuple([adjNode[0],adjNode[1]])] = tuple([currNode[0],currNode[1]])
                    break 
                print('END---->>> ')   
            # open node: add to queue                           
            elif maze[adjNode[0],adjNode[1]] == 0:    
                # Weighting fastest path
                if adjNode[2] < nodeDistances[adjNode[0],adjNode[1]]:
                    nodeDistances[adjNode[0],adjNode[1]] = adjNode[2]
                    if queueflag == False:
                        pathEdges[tuple([adjNode[0],adjNode[1]])] = tuple([currNode[0],currNode[1]])
                    print('0 New edge {}: {} | New dist: {} | Old Dist: {}'.format([adjNode[0],adjNode[1]],[currNode[0],currNode[1]],adjNode[2],nodeDistances[adjNode[0],adjNode[1]]))

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
  
                modifiedNodes.append([adjNode[0],adjNode[1]])   # Plotting stuff
                modifiedNodeStatus.append(4)

            elif maze[adjNode[0],adjNode[1]] == 4:
                if adjNode[2] < nodeDistances[adjNode[0],adjNode[1]]:
                    print('4 New edge {}: {} | New dist: {} | Old Dist: {}'.format([adjNode[0],adjNode[1]],[currNode[0],currNode[1]],adjNode[2],nodeDistances[adjNode[0],adjNode[1]]))
                    nodeDistances[adjNode[0],adjNode[1]] = adjNode[2]
                    if queueflag == False:
                        pathEdges[tuple([adjNode[0],adjNode[1]])] = tuple([currNode[0],currNode[1]])

            print('Node: {} | Adj: {} | existingDist: {}'.format(currNode,adjNode,nodeDistances[adjNode[0],adjNode[1]]))
        maze[currNode[0],currNode[1]] = 5   # mark current as Visitied
        modifiedNodes.append([currNode[0],currNode[1]]) # Plotting stuff
        modifiedNodeStatus.append(5)
                
        if depthCounter%1 == 0:
            Plot_Search_Path(modifiedNodes, modifiedNodeStatus)
            #Draw_Maze_Innit(maze)
            modifiedNodes = []
            modifiedNodeStatus = []

    #shortestPath = [tuple([adjNode[0],adjNode[1]])]
    shortestPath = [tuple(list(pathEdges)[-1])]
    while True:
        if shortestPath[-1] in pathEdges:
            shortestPath.append(pathEdges[shortestPath[-1]])
        else:
            break

    calcTime = time.time()-startTime
    print('Search: {} seconds'.format(calcTime))
    
    Plot_Search_Path(modifiedNodes, modifiedNodeStatus)
    #Draw_Maze_Innit(maze)

    print(nodeDistances)
    return shortestPath

def Draw_Maze_Innit(mazelist):
    # Loop over all points in maze
    ax.cla()
    nodeWallx = []
    nodeWally = []
    rowCounter = 0
    entryCounter = 0
    for row in mazelist:
        for entry in row:       
            # Plotting maze outline and POI
            if entry == 1:      # Obstacle
                nodeWallx.append(entryCounter)
                nodeWally.append(height-rowCounter)
                #ax.plot(entryCounter, height-rowCounter, 'ks')
            elif entry == 2:    # Start
                ax.plot(entryCounter, height-rowCounter, 'bo')
            elif entry == 3:    # End
                ax.plot(entryCounter, height-rowCounter, 'go')
            elif entry == 4:    # Queued
                ax.plot(entryCounter, height-rowCounter, 'm.')
            elif entry == 5:    # Visited
                ax.plot(entryCounter, height-rowCounter, 'c.')
            entryCounter += 1
        rowCounter += 1
        entryCounter = 0
    plt.scatter(nodeWallx,nodeWally,c='k',marker=',')
    ax.axis('equal')
    plt.pause(1e-10)
    return

def Plot_Search_Path(points, stati):    
    nodeQx = []
    nodeQy = []
    nodeVx = []
    nodeVy = []
    for nodeNum in range(len(points)):
        if stati[nodeNum] == 4:    # Queued
            nodeQx.append(points[nodeNum][1])
            nodeQy.append(height-points[nodeNum][0])
        elif stati[nodeNum] == 5:    # Visited          
            nodeVx.append(points[nodeNum][1])
            nodeVy.append(height-points[nodeNum][0])

    plt.scatter(nodeQx,nodeQy,c='m',marker='.')
    plt.scatter(nodeVx,nodeVy,c='c',marker='.')
    plt.pause(1e-10)
    return

def PlotPath(path):
    xcoords = []
    ycoords = []
    for node in path:
        ycoords.append(height-node[0])
        xcoords.append(node[1])

    plt.plot(xcoords,ycoords,'r-')
    return


if __name__ == '__main__':
    mazeList = pd.read_csv("Homework 1\SmallMap.csv", header=None).to_numpy()
    
    height, width = mazeList.shape
    nodeDistances = float(16384)*np.ones(mazeList.shape, dtype=int)
    start = np.where(mazeList==2)
    startLoc = np.array([start[0][0],start[1][0]])

    # Start interactive plot
    plt.ion()
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111)

    Draw_Maze_Innit(mazeList)
    shortestPath = Dijkstra(mazeList, [startLoc[0],startLoc[1],0])
    PlotPath(shortestPath)

    # Keeping the updating plot open
    while True:
        plt.pause(10)