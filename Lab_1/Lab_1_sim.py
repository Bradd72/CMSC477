import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import time
import random

from pupil_apriltags import Detector
import cv2
from robomaster import robot
from robomaster import camera

random.seed('CMSC477',version=2)

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

    plt.plot(xcoords,ycoords,'r:')
    plt.pause(1e-10)
    return

def ExpandWalls(maze,padding):
    '''
    Creates a 1 unit wide boundary around all known walls
    '''
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

def Draw_Maze(mazelist, grid=0):
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
    if grid == 1:
        ax.grid()
        ax.set_xticks(range(0, width))
        ax.set_xticklabels(range(1, width+1))
        ax.set_yticks(range(0, height))
        ax.set_yticklabels(range(1, height+1))
    calcTime = time.time()-startTime
    print('Plot: {} seconds'.format(calcTime))
    plt.pause(1e-10)
    return

def FollowPath(shortestPath,robotLoc):
    pathDes = shortestPath
    timeConst = 0.25 # seconds between nodes
    endFlag = False
    '''
    - get current robot position [x, y, rot]
    - get desired robot position [x, y, rot]
        - convert to world coordinates
        - interpolate path
            - time constant between nodes (two different times for diagonal and manhattan)
    - PID move to follow desired position
    '''
    K_p = 4
    K_i = .5
    K_d = .3

    x_val=0
    y_val=0
    x_integrator = 0
    y_integrator = 0
    x_diff = 0
    y_diff = 0
    prev_time = time.time()

    while endFlag == False:
        
        node1Time = time.time()
        firstNode = pathDes.pop(0)
        prevdesLoc = firstNode
        if mazeList[firstNode[0],firstNode[1]] == 3:
            pathDes.append(firstNode)
        else:
            secondNode = pathDes[0]
            nodeOffset = [secondNode[0]-firstNode[0],secondNode[1]-firstNode[1]]
            if abs(nodeOffset[0])+abs(nodeOffset[1]) > 1: # making diagonals take equal time to edges
                nodeMultiplier = np.sqrt(2)
            else:
                nodeMultiplier = 1

        tElapse = time.time() - node1Time
        while tElapse < timeConst*nodeMultiplier:
            desLoc = [firstNode[0]+nodeOffset[0]*tElapse/(timeConst*nodeMultiplier),firstNode[1]+nodeOffset[1]*tElapse/(timeConst*nodeMultiplier)]
            
            # move robot 'nodeSize'*nodeMultiplier meters in timeConst*nodeMultiplier seconds
            # PID track desLoc
            '''
            SET prevBotloc TO UPDATING REAL ROBOT POSITION
            AND DELETE RANDOM MULTIPLIERS
            ''' 
            prevBotloc = robotLoc
            x_prev = x_val
            y_prev = y_val
            x_val = robotLoc[0]+5*nodeSize*(random.random()-0.5)-desLoc[0]
            y_val = robotLoc[1]+5*nodeSize*(random.random()-0.5)-desLoc[1]
            time_step = time.time() - prev_time
            prev_time = time.time()
            if time_step < .5 and time_step != 0:
                x_integrator+=x_val
                y_integrator+=y_val
                x_diff = (x_val - x_prev) / time_step
                y_diff = (y_val - y_prev) / time_step
            else:
                x_integrator = 0
                y_integrator = 0
                x_diff = 0
                y_diff = 0
            '''
            INPUT YAW ANGLE TO ROTATE, CURRENT PATH DOES NOT INCORPORATE ROTATIONS
            '''
            angle =  np.rad2deg(np.arctan2(y_val,x_val))
            if abs(angle)>15 and x_val>.2:
                z_val = angle
                y_val = 0
            else:
                z_val=0
            x_response = x_val*K_p + x_diff*K_d + x_integrator*K_i
            y_response = y_val*K_p + y_diff*K_d + y_integrator*K_i
            z_response = z_val*K_p
            #ep_chassis.drive_speed(x_response, y_response, z_response,timeout=.1)
            prevBotloc = [robotLoc[0],robotLoc[1]]
            robotLoc[0] = robotLoc[0]-x_response*time_step
            robotLoc[1] = robotLoc[1]-y_response*time_step
            ax.plot([prevBotloc[1],robotLoc[1]],[height-prevBotloc[0],height-robotLoc[0]],'c')
            #ax.plot(robotLoc[1],height-robotLoc[0],"c.")
            print("x: {} | y: {}".format(robotLoc[1],robotLoc[0]))

            ax.plot([prevdesLoc[1],desLoc[1]],[height-prevdesLoc[0],height-desLoc[0]],'g')
            plt.pause(0.01)
            prevdesLoc = desLoc
            tElapse = time.time() - node1Time
    return

if __name__ == '__main__':
    # ep_robot = robot.Robot()
    # ep_robot.initialize(conn_type="ap")
    # ep_chassis = ep_robot.chassis
    

    nodeSize = 16.6/5/100 # Box width in cm with maze resolution at 5 converted to m
    # FOR INITIAL LAB WORK, KNOWN ENVIRONMENT
    '''
    - |X| init maze
    - | | determine current location in maze
        - | | Apriltag distances give relative location
        - | | Apriltag id gives location in map
    - |X| solve path
    - | | follow path
        - | | FIND A WAY TO DETERMINE NEXT POINTS/PATH
        - | | interpolate between next point to find desired location given a certain time or velocity between
        - | | PID controller to move robot to point moving in space (in time!)
            - | | Use other apriltags to determine position in maze and calculate error offset
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
    ExpandWalls(mazeList,padding=4)
    Draw_Maze(mazeList,grid=0)

    # Determine current location in maze
    robotLoc = startLoc
    
    # Solve Path
    shortestPath = Dijkstra(mazeList, [robotLoc[0],robotLoc[1],0])
    PlotPath(shortestPath)
    print(shortestPath)

    # Follow Path
    ax.plot(robotLoc[1],height-robotLoc[0],'mx')
    robotLoc = [float(robotLoc[0]),float(robotLoc[1])]
    FollowPath(shortestPath,robotLoc)

    # Keeping the updating plot open
    while True:
        plt.pause(1e-10)


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