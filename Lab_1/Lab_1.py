from pupil_apriltags import Detector
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from robomaster import robot
from robomaster import camera
import pandas as pd
import time
import matplotlib.pyplot as plt
import random

MAZE_TO_METERS = .0536
METERS_TO_MAZE = 1/MAZE_TO_METERS

robot_coord = [0,0,0]
marker_List = pd.read_csv("Lab_1\WallLookUp.csv", header=None).to_numpy()

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

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

def marker_transform(xcoord,ycoord,deg_angle):
    temp_rot = np.eye(3)*-1
    temp_rot[2,2] = 1
    rot, jaco = cv2.Rodrigues(np.array([ 0,np.deg2rad(deg_angle),0]))
    transform = np.eye(4)
    transform[:3,:3] = np.matmul(temp_rot,rot)
    transform[0,3] = xcoord*MAZE_TO_METERS
    transform[2,3] = ycoord*MAZE_TO_METERS
    transform[1,3] = .15
    return transform

def pose_transform(pose):
    temp_rot = np.eye(3)*-1
    temp_rot[2,2] = 1
    rot, jaco = cv2.Rodrigues(pose[1])
    transform = np.eye(4)
    transform[:3,:3] = np.matmul(rot,temp_rot)
    transform[0,3] = pose[0][0] *1
    transform[1,3] = pose[0][1] *1
    transform[2,3] = pose[0][2] *1
    return np.linalg.inv(transform)

def find_pose_from_tag(K, detection):
    m_half_size = tag_size / 2

    marker_center = np.array((0, 0, 0))
    marker_points = []
    marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
    marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
    _marker_points = np.array(marker_points)

    object_points = _marker_points
    image_points = detection.corners

    pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if pnp_ret[0] == False:
        raise Exception('Error solving PnP')

    r = pnp_ret[1]
    p = pnp_ret[2]

    return p.reshape((3,)), r.reshape((3,))

if __name__ == '__main__':
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    nodeSize = 16.6/5/100 # Box width in cm with maze resolution at 5 converted to m
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
    #print(shortestPath)

    # Follow Path
    ax.plot(robotLoc[1],height-robotLoc[0],'mx')

    K_p = .5
    K_i = .01
    prog_time=time.time()
    time_=prog_time
    x_error=0
    z_error=0
    head_error=0
    x_integrator = 0
    z_integrator = 0
    head_integrator=0

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    tag_size=0.16 # tag size in meters
    
    endFlag = False
    pathDes = shortestPath
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
        error_tol=.1
        error_norm=error_tol+1
        # TODO what is this
        desLoc = [firstNode[0]+nodeOffset[0]*tElapse/(timeConst*nodeMultiplier),firstNode[1]+nodeOffset[1]*tElapse/(timeConst*nodeMultiplier)]
            
        while error_norm>error_tol:
            try:
                img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                gray.astype(np.uint8)

                K=np.array([[184.752*1.7, 0, 320], [0, 184.752*1.7, 180], [0, 0, 1]])

                results = at_detector.detect(gray, estimate_tag_pose=False)

                x_estimation=[]
                z_estimation=[]
                rotation_estimation=[]
                weights=[]
                
                for res in results:
                    pose = find_pose_from_tag(K, res)
                    pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                    
                    if (abs(pose[0][0])>.75):
                        img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                        cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                        continue
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 255, 0), thickness=5)
                    cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                    T_april_cam = pose_transform(pose)
                    
                    for i in range(len(marker_List)):
                        if marker_List[i][0] == res.tag_id:
                            T_globe_april = marker_transform(marker_List[i,1],marker_List[i,2],1*(marker_List[i,3]))
                            T_globe_cam = np.matmul(T_globe_april,T_april_cam)
                            rot_ = np.matmul([1,0,0],T_globe_cam[:3,:3])
                            rotation_estimation.append(np.arctan2(rot_[2],rot_[0])*-1)
                            x_estimation.append(T_globe_cam[0,3])
                            z_estimation.append(T_globe_cam[2,3])
                            # weights.append(1)
                            weights.append(abs(T_globe_cam[0,3])**-1)        
                if x_estimation != []:
                    robot_coord = [np.average(x_estimation,weights=weights),np.average(z_estimation,weights=weights),np.average(rotation_estimation,weights=weights)]
                # print("{} {} {}".format(res.tag_id,pose[0],pose[1]))
                print("{:.3f},{:.3f}    {:.2f},{:.2f} rot={:.2f}   {:.2f}, {:.2f}, {:.2f} ".format(robot_coord[0],robot_coord[1],robot_coord[0]*METERS_TO_MAZE,robot_coord[1]*METERS_TO_MAZE,np.rad2deg(robot_coord[2]),np.var(x_estimation),np.var(z_estimation),np.var(rotation_estimation)))
                
                x_error = desLoc[0]*MAZE_TO_METERS - robot_coord[0]
                z_error = desLoc[0]*MAZE_TO_METERS - robot_coord[1]
                head_error = heading_maze_des-robot_coord[2]
                error_norm = np.linalg.norm(np.array([x_error,z_error,head_error*10]))
                prev_time = time_
                time_step = time_ - prev_time
                if time_step < .5:
                    x_integrator+=x_error
                    z_integrator+=z_error
                    head_integrator+=head_error
                else:
                    x_integrator = 0
                    z_integrator = 0
                    head_integrator=0

                bot_x_response = np.cos(robot_coord[2])*(K_p*z_error+K_i*z_integrator)+np.sin(robot_coord[2])*(K_p*x_error+K_i*x_integrator)
                bot_y_response = np.sin(robot_coord[2])*(K_p*z_error+K_i*z_integrator)+np.cos(robot_coord[2])*(K_p*x_error+K_i*x_integrator)
                bot_z_response = K_p*(head_error)+K_i*(head_integrator)
                print("{:.3f}, {:.3f}, {:.3f}".format(bot_x_response,bot_y_response,bot_z_response))
                ep_chassis.drive_speed(bot_x_response*-1, bot_y_response*1, bot_z_response*-20,timeout=.1)
                # ep_chassis.drive_speed(0, .1, 0,timeout=.1)
                cv2.imshow("img", img)
                cv2.waitKey(10)

            except KeyboardInterrupt:
                ep_camera.stop_video_stream()
                ep_robot.close()
                print ('Exiting')
                exit(1)
