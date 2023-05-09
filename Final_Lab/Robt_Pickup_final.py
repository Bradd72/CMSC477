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

# Centroid PID Globals
K_p_pix=.0005; K_i_pix=.000002; K_d_pix=0; cX=0; cY=0
error_norm_pix=np.ones((20,1))*100;error_tol_pix=15
y_pix_diff=0;x_pix_diff=0;y_pix_integrator=0;x_pix_integrator=0;y_pix_error=0;x_pix_error=0;y_response=0;x_response=0;error_count_pix=0;prev_time=time.time()
# Map PID Globals
K_p = .35;K_i = .25;K_d=0.1;K_p_z = 1;K_i_z = .5;K_d_z = 0
initial_x=0;initial_y=0;initial_head=0
x_error=0;x_diff=0;x_integrator=0;x_response=0;est_x=0
y_error=0;y_diff=0;y_integrator=0;y_response=0;est_y=0
head_error=0;head_diff=0;head_integrator=0;z_response=0;est_heading=0
error_norm=np.ones((50,1))*100;error_tol=.025;error_count=0
prev_time=time.time()
yaw=initial_head;robo_x=initial_x;robo_y=initial_y
METERS_TO_MAP = 15



def sub_attitude_info_handler(attitude_info):
    global yaw
    yaw = attitude_info[0]


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

def sub_distance_handler(dist_info):
    global ir_distance_m
    ir_distance_m = dist_info[0]/1000

def sub_position_handler(position_info):
    global est_x, est_y
    robo_x, robo_y, robo_z = position_info
    est_x = robo_x + initial_x
    est_y = robo_y + initial_y


def find_closet_block():
    yel_range = (np.array([20,120,100]),np.array([50,255,255]),"yellow")
    org_range = (np.array([10,113,180]),np.array([22,255,255]),"orange")
    green_range = (np.array([65,105,75]),np.array([83,255,255]),"green")
    red_range = (np.array([0,125,150]),np.array([7,255,255]),"red")
    blue_range = (np.array([90,50,76]),np.array([135,255,255]),"blue")
    item_found = False
    greatest_y=0;best_range=org_range
    for color_range in [org_range,green_range,red_range,blue_range,yel_range]:
        for iter_num in range(20):
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)  
            blurred = cv2.medianBlur(frame,9)
            output = blurred.copy() 
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, color_range[0], color_range[1])
            (totalLabels, label_ids, values, centroid) = cv2.connectedComponentsWithStats(mask,4,cv2.CV_32S)
            for i in range(1, totalLabels):
                    w = values[i, cv2.CC_STAT_WIDTH]
                    h = values[i, cv2.CC_STAT_HEIGHT]
                    area = values[i, cv2.CC_STAT_AREA] 
                    # print("area: {}  w:{} h:{}".format(area,w,h))
                    # Checks if Item is big enough and if looking for orange will make sure it is wider than tall
                    if (area > 750) and (area < 10000):
                        item_found = True
                        componentMask = (label_ids == i).astype("uint8") * 255
                        if centroid[i][1] > greatest_y:
                            greatest_y = centroid[i][1]
                            best_range = color_range
                        cv2.circle(frame, (int(centroid[i][0]), int(centroid[i][1])), 4, (0, 0, 255), -1)
            # cv2.imshow("out",output)
    # print(best_range,greatest_y)
    if item_found:
        print("Going for {} block".format(best_range[2]))
        return best_range
    else:
        return False

def pickup_block(color_range):
    global cX,cY
    run_bool = True
    greatest_y=0
    while(run_bool):
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)  
        blurred = cv2.medianBlur(frame,9)
        output = blurred.copy() 
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, color_range[0], color_range[1])
        (totalLabels, label_ids, values, centroid) = cv2.connectedComponentsWithStats(mask,4,cv2.CV_32S)
        for i in range(1, totalLabels):
                w = values[i, cv2.CC_STAT_WIDTH]
                h = values[i, cv2.CC_STAT_HEIGHT]
                area = values[i, cv2.CC_STAT_AREA] 
                # print("area: {}  w:{} h:{}".format(area,w,h))
                # Checks if Item is big enough and if looking for orange will make sure it is wider than tall
                if (area > 750) and (area < 10000):
                    item_found = True
                    componentMask = (label_ids == i).astype("uint8") * 255
                    if centroid[i][1] > greatest_y:
                        greatest_y = centroid[i][1]
                        best_range = color_range
                        (cX, cY) = centroid[i]
                    cv2.rectangle(img=output, pt1=(round(cX-w/2),round(cY-h/2)),
                                        pt2=(round(cX+w/2),round(cY+h/2)),
                                        color=(0,255,255), thickness=2)
        cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)
        # print("{}, {}".format(cX,cY))
        greatest_y=max(1,greatest_y-20)

        if centroid_pid(320,225):
            ep_chassis.drive_speed(.125,0,0,timeout=.25);time.sleep(.25)
            ep_chassis.drive_speed(0,0,0);time.sleep(.1)
            ep_gripper.close();time.sleep(2);ep_gripper.stop()
            ep_arm.move(0,50).wait_for_completed()
            break

        ep_chassis.drive_speed(x_response*-1, y_response*1,0,timeout=.5)


        cv2.imshow("out",output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def centroid_pid(des_cX,des_cY):
    global y_pix_diff,x_pix_diff,y_pix_integrator,x_pix_integrator,y_pix_error,x_pix_error,y_response,x_response,error_count_pix,error_norm_pix, prev_time
    y_pix_prev=y_pix_error
    x_pix_prev=x_pix_error
    y_pix_error = (cX - des_cX)
    x_pix_error = (cY - des_cY)
    time_ = time.time()
    time_step = time_ - prev_time
    prev_time = time_
    if 0 < time_step < .5:
        y_pix_integrator +=y_pix_error  
        y_pix_diff = (y_pix_error - y_pix_prev) / time_step
        x_pix_integrator +=x_pix_error  
        x_pix_diff = (x_pix_error - x_pix_prev) / time_step
    else:
        y_pix_integrator = 0;y_pix_diff = 0;x_pix_integrator = 0;x_pix_diff = 0
    
    y_response = y_pix_error*K_p_pix+y_pix_integrator*K_i_pix+y_pix_diff*K_d_pix
    
    x_response = x_pix_error*K_p_pix+x_pix_integrator*K_i_pix+x_pix_diff*K_d_pix
    if error_count_pix>=len(error_norm_pix)-1:
        error_count_pix=0
    else:
        error_count_pix+=1
    error_norm_pix[error_count_pix]=np.linalg.norm((x_pix_error,y_pix_error))
    print("{:.2f},{:.2f}  err:{:.2f}".format(x_response,y_response,np.mean(error_norm_pix)))
    if np.mean(error_norm_pix)<error_tol_pix: #
        error_norm_pix=np.ones((20,1))*100;y_pix_integrator = 0;y_pix_diff = 0;x_integrator = 0;x_diff = 0
        return True
    else:
        return False


if __name__ == '__main__':
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
    ep_arm.moveto(180,-80).wait_for_completed()
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    #x = threading.Thread(target=move_square, daemon=True, args=(ep_robot.chassis,))
    #x.start()
    
    run_bool=True
    framecount = 1
    while(run_bool):
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
        frame_height, frame_width, c = frame.shape
        output = frame.copy() 
        # Other stuff
        plt.plot(est_x*METERS_TO_MAP,-est_y*METERS_TO_MAP,'m.')
        #print("d:%5.2f | y: %5.2f" % (ir_distance_m, yaw))
        objectLoc = [x_new[0]+np.cos(np.deg2rad(yaw))*ir_distance_m,x_new[0]+np.sin(np.deg2rad(yaw))*ir_distance_m]
        if (ir_distance_m <= 3):
            if (objectLoc[0] >= 0 and objectLoc[1] >= 0 and int(15*objectLoc[1]) < height and int(15*objectLoc[0]) < width):
                #plt.plot(int(15*(objectLoc[0])),int(-15*(objectLoc[1])),c='r',marker='x')
                #mazeList[int(15*(objectLoc[1])),int(15*(objectLoc[0]))] = 9     # mazeList[y,x]
                Dijkstra.SetObstacles(mazeList,[int(15*objectLoc[0]),int(15*objectLoc[1])],2)
        cv2.imshow("out",output)
        x_old = np.copy(x_new)

        if (framecount%250 == 0):
            plt.cla()
            print("Full Clear")
            Dijkstra.RemoveObstacles(mazeList)
            Dijkstra.Draw_Maze(mazeList,ax)
            framecount = 0
        elif (framecount%25 == 0):
            plt.cla()
            print("Clear")
            Dijkstra.Draw_Maze(mazeList,ax)
            # Solve Path
            if (int(METERS_TO_MAP*est_x) < 1 or int(METERS_TO_MAP*est_y) < 1):
                pathDes = Dijkstra.Dijkstra(mazeList, [oldxloc[0],oldxloc[1],0])
            else:
                pathDes = Dijkstra.Dijkstra(mazeList, [int(METERS_TO_MAP*est_x),int(METERS_TO_MAP*est_y),0])
                oldxloc = [est_x,est_y]
            Dijkstra.PlotPath(pathDes)
        #time.sleep(1)
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