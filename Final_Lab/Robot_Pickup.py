'''
CMSC477 - Final Lab
Robot responsible for picking and transfering blocks over river

Left off: 

'''
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import time
import pandas as pd
import matplotlib.pyplot as plt
import random
import threading

random.seed('Final Lab',version=2)

import Dijkstra

from roboflow import Roboflow
#rf = Roboflow(api_key="kKusTXhj0ObVGmi9slHp")
#project = rf.workspace().project("project2-l7rdy")
#model = project.version(4).model

import socket
from socket import gethostbyname
import select
HOST = gethostbyname('0.0.0.0')
#HOST = 'localhost'
PORT = 65439
ACK_TEXT = 'text_received'

def sub_attitude_info_handler(attitude_info):
    global yaw, pitch, roll
    yaw, pitch, roll = attitude_info

def sub_data_handler(sub_info):
    global pos_x, pos_y 
    pos_x, pos_y = sub_info

def sendTextViaSocket(message, sock):
    encodedMessage = bytes(message, 'utf-8')        # encode the text message
    sock.sendall(encodedMessage)                    # send the data via the socket to the server
    encodedAckText = sock.recv(1024)                # receive acknowledgment from the server
    ackText = encodedAckText.decode('utf-8')

    # log if acknowledgment was successful
    if ackText == ACK_TEXT:
        print('Server acknowledged reception')
    else:
        print('Error: Server returned ' + ackText)
    
    return

def sub_position_handler(p, x_new):
    x_new[0] = p[0]
    x_new[1] = p[1]
    x_new[2] = p[2]
    # print("chassis position: x: {}".format(x_new))

wait_to_start_moving = True
def move_square(ep_chassis, x_len=0.5, y_len=0.5, speed=1.0):
    while wait_to_start_moving: time.sleep(0.1)
    while True:
        ep_chassis.move(x=x_len,  y=0,      z=0, xy_speed=speed).wait_for_completed()
        ep_chassis.move(x=0,      y=y_len,  z=0, xy_speed=speed).wait_for_completed()
        ep_chassis.move(x=-x_len, y=0,      z=0, xy_speed=speed).wait_for_completed()
        ep_chassis.move(x=0,      y=-y_len, z=0, xy_speed=speed).wait_for_completed()

def sub_distance_handler(dist_info):
    global ir_distance
    ir_distance = dist_info[0]


if __name__ == '__main__':
    useRobot = True

    # # Open Socket Communication Server
    # print('Initializing Socket...')
    # sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # instantiate a socket object
    # print('Binding To Socket...')
    # sock.bind((HOST, PORT))                                     # bind the socket
    # sock.listen()                                               # start the socket listening
    # print('Listening To Socket...')

    # # accept the socket response from the client, and get the connection object
    # conn, addr = sock.accept()      # Note: execution waits here until the client calls sock.connect()
    # print('Socket Connection Accepted, Received Connection Object')

    # message = input("Message to send: ")
    # print('Sending: ' + message)
    # sendTextViaSocket(message, conn)

    mazeList = pd.read_csv("Labs\Final_Lab\Final_Lab_maze2.csv", header=None).to_numpy() # mazelist[y,x]
    height, width = mazeList.shape
    mazeList[55,10] = 2
    mazeList[55,5] = 3     # mazeList[y,x]

    start = np.where(mazeList==2)
    startLoc = np.array([start[1][0],start[0][0]])
    oldxloc = [startLoc[0],startLoc[1],0]
    # Start interactive plot
    plt.ion()
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111)
    Dijkstra.ExpandWalls(mazeList,padding=3)
    Dijkstra.Draw_Maze(mazeList,ax)
    # Solve Path
    shortestPath = Dijkstra.Dijkstra(mazeList, [startLoc[0],startLoc[1],0])
    Dijkstra.PlotPath(shortestPath)
    plt.pause(1)

    pathDes = shortestPath
    timeConst = 0.25 # seconds between nodes
    
    if (useRobot):
        x_old = np.zeros((3,))
        x_new = np.zeros((3,))
        
        # Initialize Robot
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")
        ep_chassis = ep_robot.chassis
        ep_camera = ep_robot.camera
        ep_gripper = ep_robot.gripper
        ep_arm = ep_robot.robotic_arm
        ep_sensor = ep_robot.sensor
        ep_robot.chassis.sub_position(freq=50, callback=lambda p: sub_position_handler(p, x_new))
        ep_sensor.sub_distance(freq=10,callback=sub_distance_handler)
        ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
        ep_gripper.open()
        ep_arm.moveto(180,-20).wait_for_completed()
        ep_arm.sub_position(freq=5, callback=sub_data_handler)
        ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

        #x = threading.Thread(target=move_square, daemon=True, args=(ep_robot.chassis,))
        #x.start()
        
        run_bool=True
        framecount = 1
        while(run_bool):
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
            frame_height, frame_width, c = frame.shape
            output = frame.copy() 

            wait_to_start_moving = False

            # Path to follow
            followPath = True
            if (followPath):
                node1Time = time.time()
                firstNode = pathDes.pop(0)
                prevdesLoc = firstNode
                if mazeList[firstNode[1],firstNode[0]] == 3:
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
                    
                    ax.plot([prevdesLoc[0],desLoc[0]],[-prevdesLoc[1],-desLoc[1]],'g')
                    plt.pause(1e-10)
                    prevdesLoc = desLoc
                    tElapse = time.time() - node1Time


            # Other stuff
            if x_old is None:
                x_old = np.copy(x_new)
            
            plt.plot(x_new[0]*15,-x_new[1]*15,'m.')
            #print("d:%5.2f | y: %5.2f" % (ir_distance/1000, yaw))
            objectLoc = [x_new[0]+np.cos(np.deg2rad(yaw))*ir_distance/1000,x_new[0]+np.sin(np.deg2rad(yaw))*ir_distance/1000]
            if (ir_distance/1000 <= 3):
                if (objectLoc[0] >= 0 and objectLoc[1] >= 0 and int(15*objectLoc[1]) < height and int(15*objectLoc[0]) < width):
                    #plt.plot(int(15*(objectLoc[0])),int(-15*(objectLoc[1])),c='r',marker='x')
                    #mazeList[int(15*(objectLoc[1])),int(15*(objectLoc[0]))] = 9     # mazeList[y,x]
                    Dijkstra.SetObstacles(mazeList,[int(15*objectLoc[0]),int(15*objectLoc[1])],2)
            cv2.imshow("out",output)
            x_old = np.copy(x_new)

            if (framecount%250 == 0):
                plt.cla()
                print("clearing")
                Dijkstra.RemoveObstacles(mazeList)
                Dijkstra.Draw_Maze(mazeList,ax)
                framecount = 0
            if (framecount%25 == 0):
                plt.cla()
                print("clearing")
                Dijkstra.Draw_Maze(mazeList,ax)
                # Solve Path
                if (int(15*x_new[0]) < 1 or int(15*x_new[1]) < 1):
                    pathDes = Dijkstra.Dijkstra(mazeList, [oldxloc[0],oldxloc[1],0])
                else:
                    pathDes = Dijkstra.Dijkstra(mazeList, [int(15*x_new[0]),int(15*x_new[1]),0])
                    oldxloc = x_new
                Dijkstra.PlotPath(pathDes)
            #time.sleep(1)
            '''
            - Locate robot in the world and pickup location
                - Rotate robot until it sees both the visual markers and pickup zone
        *    - requires visual odometry calculations from camera
                        - Gets robots location in the world
                        - Find obstacles in the world
                - distinguishing the 8020 from the floor
            '''

            '''
            - Go to and pickup block
        *- Roboflow (locally train) on the Robot and maybe block/river
                    - PID to block + grab/lift based on centroid and relative size in camera fov
            '''

            '''
        *- Navigate to the river
                - CV2 for river centroid, PID to location
            '''

            '''
        *- Handoff Block (Joint effort)
                - Go to world coordinate location
                - Locate other robot and align with it
                - Move forward and communicate states for block handoff
            '''
            framecount += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # Destroys all of the windows and closes camera  
        print ('Exiting')
        ep_chassis.drive_speed(0,0,0,timeout=.1)
        time.sleep(1)
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()
        exit(1)