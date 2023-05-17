'''
CMSC477 - Final Lab
Robot responsible for picking and transfering blocks over river

TODO: error when robot is inside of the keepout-zone and it tries to solve a path

'''
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import time
import pandas as pd
import matplotlib.pyplot as plt
import Dijkstra

# Map Gobals
METERS_TO_MAP = 15
# TEAM = "bot"
TEAM = "top"
river_center = True
if TEAM=="top":
    ANGLE_SIGN = 1
    INITIAL_LOC = (4/3.281,2/3.281)
    if river_center:
        BLOCK_PICKUP_LOC = (6/3.281+.114,6.5/3.281)
    else:
        BLOCK_PICKUP_LOC = (6/3.281,6.5/3.281)
    BLOCK_PLACE_LOC = (2/3.281,3.25/3.281)
    
elif TEAM=="bot":
    ANGLE_SIGN = -1
    INITIAL_LOC = (4/3.281,12/3.281)
    if river_center:
        BLOCK_PICKUP_LOC = (6/3.281+.114,8/3.281)
    else:
        BLOCK_PICKUP_LOC = (6/3.281,8/3.281)
    BLOCK_PLACE_LOC = (2/3.281,10.75/3.281)
    
# Centroid PID Globals
K_p_pix=.00075; K_i_pix=.000004; K_d_pix=0; cX=0; cY=0
error_norm_pix=np.ones((20,1))*100;error_tol_pix=15
y_pix_diff=0;x_pix_diff=0;y_pix_integrator=0;x_pix_integrator=0;y_pix_error=0;x_pix_error=0;y_response=0;x_response=0;error_count_pix=0;prev_time=time.time()
# Map PID Globals
K_p = 2;K_i = .25;K_d=0.2
K_p_z = 1.5;K_i_z = 0;K_d_z = 0
initial_x=INITIAL_LOC[0];initial_y=INITIAL_LOC[1];initial_head=0
yaw_adjustment=0
x_error=0;x_diff=0;x_integrator=0;x_response=0;est_x=0
y_error=0;y_diff=0;y_integrator=0;y_response=0;est_y=0
head_error=0;head_diff=0;head_integrator=0;z_response=0
error_norm=np.ones((25,1))*25;error_count=0;error_tol=.15
prev_time=time.time()
est_heading=initial_head;est_heading_rad=np.deg2rad(est_heading);est_x=initial_x;est_y=initial_y
tangent_angle_rad=initial_head

def sub_attitude_info_handler(attitude_info):
    global est_heading_rad,initial_head,est_heading
    yaw = -attitude_info[0] - yaw_adjustment
    est_heading=yaw+initial_head;est_heading_rad=np.deg2rad(est_heading)

def sub_distance_handler(dist_info):
    global ir_distance_m
    ir_distance_m = dist_info[0]/1000

def sub_position_handler(position_info):
    global est_x, est_y
    robo_x, robo_y, robo_z = position_info
    est_x = robo_x + initial_x
    est_y = robo_y + initial_y

def odom_pid(des_X,des_Y,des_head,tight=False):
    
    global y_diff,x_diff,head_diff,y_integrator,x_integrator,head_integrator,y_error,x_error,head_error,y_response,x_response,z_response,error_count,error_norm,prev_time
    y_prev=y_error
    x_prev=x_error
    head_prev=head_error
    head_error = (est_heading - des_head)
    if abs(head_error) > 180:
        head_error = np.sign(des_head)*(360-abs(est_heading-des_head))
    y_error = (est_y - des_Y)*1
    x_error = (est_x - des_X)*1
    time_ = time.time()
    time_step = time_ - prev_time
    prev_time = time_
    if 0<time_step and time_step < .5 and tight:
        y_integrator +=y_error*time_step   
        y_diff = (y_error - y_prev) / time_step
        x_integrator +=x_error*time_step  
        x_diff = (x_error - x_prev) / time_step
        head_integrator +=head_error*time_step  
        head_diff = (head_error - head_prev) / time_step
        
    else:
        # print("long wait: {}".format(time_step))
        y_integrator = 0; y_diff = 0; x_integrator = 0; x_diff = 0; head_integrator=0; head_diff=0
    y_response = y_error*K_p+y_integrator*K_i+y_diff*K_d
    x_response = x_error*K_p+x_integrator*K_i+x_diff*K_d
    if x_error>.5:
        x_response = x_error*K_p
    if y_error>.5:
        y_response = y_error*K_p
    # z_response = head_error*K_p_z+head_integrator*K_i_z+head_diff*K_d_z
    if abs(head_error)<20:
        z_response = head_error*K_p_z+head_integrator*K_i_z+head_diff*K_d_z
    else:
        z_response = (head_error*K_p_z+head_integrator*K_i_z+head_diff*K_d_z)*.5
    if error_count>=len(error_norm)-1:
        error_count=0
    else:
        error_count+=1
    print("({:.2f},{:.2f},{:.2f})  ({:.2f},{:.2f},{:.2f})  maze:({:.2f}, {:.2f})".format(est_x,est_y,est_heading,des_X,des_Y,des_head,est_x*METERS_TO_MAP,-est_y*METERS_TO_MAP))
    error_norm[error_count]=np.linalg.norm((x_error,y_error,head_error*.125))
    if tight:
        if np.mean(error_norm)<error_tol*.25: #
            error_norm=np.ones((25,1))*25
            return True
        else:
            return False
    else:
        if np.mean(error_norm)<error_tol: #
            error_norm=np.ones((25,1))*25
            return True
        else:
            return False

def find_closet_block():
    global cX,cY
    yel_range = (np.array([20,120,100]),np.array([50,255,255]),"yellow")
    org_range = (np.array([10,113,180]),np.array([22,255,255]),"orange")
    green_range = (np.array([65,105,75]),np.array([83,255,255]),"green")
    red_range = (np.array([0,125,150]),np.array([7,255,255]),"red")
    # blue_range = (np.array([90,50,76]),np.array([135,255,255]),"blue")
    blue_range = (np.array([100,140,40]),np.array([135,255,255]),"blue")
    item_found = False
    greatest_y=90;best_range=org_range
    while not item_found:
        for color_range in [org_range,green_range,red_range,yel_range]:
            iter_num=0
            while iter_num<5:
                # frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)  
                try:
                    frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)  
                except:
                    print("Camera Fail {}".format(iter_num))
                    iter_num = iter_num-2 if iter_num>=4 else iter_num
                    continue
                iter_num+=1
                blurred = cv2.medianBlur(frame,9)

                output = blurred.copy() 
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
                # half_mask = np.zeros(frame.shape[:2], dtype="uint8");half_mask[,]
                mask = cv2.inRange(hsv, color_range[0], color_range[1])
                (totalLabels, label_ids, values, centroid) = cv2.connectedComponentsWithStats(mask,4,cv2.CV_32S)
                for i in range(1, totalLabels):
                        w = values[i, cv2.CC_STAT_WIDTH]
                        h = values[i, cv2.CC_STAT_HEIGHT]
                        area = values[i, cv2.CC_STAT_AREA] 
                        # print("area: {}  w:{} h:{}".format(area,w,h))
                        # Checks if Item is big enough and if looking for orange will make sure it is wider than tall
                        if (area > 750) and (area < 10000):
                            componentMask = (label_ids == i).astype("uint8") * 255
                            if centroid[i][1] > greatest_y:
                                item_found = True
                                print("y:{:.2f} c:{}".format(greatest_y,color_range[2]))
                                greatest_y = centroid[i][1]
                                (cX, cY) = centroid[i]
                                best_range = color_range
                            cv2.circle(output, (int(centroid[i][0]), int(centroid[i][1])), 4, (0, 0, 255), -1)
            
    # print(best_range,greatest_y)
    if item_found:
        print("Going for {} block".format(best_range[2]))
        return best_range
    else:
        print("no block found")
        return False

def pickup_block(color_range):
    global cX,cY
    run_bool = True
    greatest_y=0
    while(run_bool):
        # frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)  
        try:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)  
        except:
            print("Camera Fail")
            continue 
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
                        (cX, cY) = centroid[i]
                    cv2.rectangle(img=output, pt1=(round(centroid[i][0]-w/2),round(centroid[i][1]-h/2)),
                                        pt2=(round(centroid[i][0]+w/2),round(centroid[i][1]+h/2)),
                                        color=(0,255,255), thickness=2)
        cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)
        # print("{}, {}".format(cX,cY))
        greatest_y=max(1,greatest_y-20)

        if centroid_pid(320,225):
            ep_chassis.drive_speed(.125,0,0,timeout=.25);time.sleep(.25)
            ep_chassis.drive_speed(0,0,0);time.sleep(.1)
            ep_gripper.close();time.sleep(2);ep_gripper.stop()
            # ep_arm.move(0,140).wait_for_completed()
            act=ep_arm.moveto(75,60);time.sleep(1);act._state='action_succeeded';ep_arm._action_dispatcher._in_progress = {}
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

def updateMapLoc(plot_bool=True,update_bool=True,update_path=False,clear_bool=False):
    if plot_bool:
        lst= [0,90,180,-180,-90]
        rounded_heading = lst[min(range(len(lst)), key = lambda i: abs(lst[i]-est_heading))]
        if rounded_heading==0:
            plt.plot(est_x*METERS_TO_MAP,-est_y*METERS_TO_MAP,'m>') 
        elif rounded_heading==90:
            plt.plot(est_x*METERS_TO_MAP,-est_y*METERS_TO_MAP,'m^') 
        elif rounded_heading==-90:
            plt.plot(est_x*METERS_TO_MAP,-est_y*METERS_TO_MAP,'mv') 
        elif abs(rounded_heading)==180:
            plt.plot(est_x*METERS_TO_MAP,-est_y*METERS_TO_MAP,'m<') 

        # if (ir_distance_m <= 2):
        if (ir_distance_m <= 1.25):
            d_x=.014;d_y=.01
            correction_vec = [d_x*np.cos(est_heading_rad)-d_y*np.sin(est_heading_rad), -1*(d_x*np.sin(est_heading_rad)+d_y*np.cos(est_heading_rad))]
            objectLoc = [est_x+np.cos(est_heading_rad)*ir_distance_m+correction_vec[0],est_y-np.sin(est_heading_rad)*ir_distance_m+correction_vec[1]]
            if (objectLoc[0] > 0 and objectLoc[1] > 0 and int(METERS_TO_MAP*objectLoc[1]) < height and int(METERS_TO_MAP*objectLoc[0]) < width):
                Dijkstra.SetObstacles(mazeList,[int(METERS_TO_MAP*objectLoc[0]),int(METERS_TO_MAP*objectLoc[1])],2)
    if (update_bool):
        plt.pause(1e-10)
    if (update_path):
        plt.cla()
        print("Full Clear")
        Dijkstra.Draw_Maze(mazeList,ax)
        Dijkstra.PlotPath(pathDes)
    if (clear_bool):
        plt.cla()
        print("Full Clear")
        Dijkstra.RemoveObstacles(mazeList)
        Dijkstra.Draw_Maze(mazeList,ax)
        Dijkstra.PlotPath(pathDes)

def responseToSpeed():
    alpha_rad = np.arctan2(y_response,x_response)
    response_mag = np.linalg.norm((x_response,y_response))
    robo_x_speed = response_mag*np.cos(alpha_rad+est_heading_rad)
    robo_y_speed = response_mag*np.sin(alpha_rad+est_heading_rad)
    return robo_x_speed,robo_y_speed

def lookAround(angle1,angle2,num_cycles=2):
    for iter in range(num_cycles):
        while not odom_pid(est_x,est_y,angle1,tight=True):
            # robo_x_speed, robo_y_speed = responseToSpeed()
            robo_x_speed=0; robo_y_speed = 0
            ep_chassis.drive_speed(-1*robo_x_speed,-1*robo_y_speed,1*z_response,timeout=.1)
            time.sleep(.01)
            updateMapLoc(update_bool=False)
        updateMapLoc()
        while not odom_pid(est_x,est_y,angle2,tight=True):
            # robo_x_speed, robo_y_speed = responseToSpeed()
            robo_x_speed=0; robo_y_speed = 0
            ep_chassis.drive_speed(-1*robo_x_speed,-1*robo_y_speed,1*z_response,timeout=.1)
            time.sleep(.01)
            updateMapLoc(update_bool=False)
        updateMapLoc()

if __name__ == '__main__':
    mazeList = pd.read_csv("Labs\Final_Lab\Final_Lab_maze2.csv", header=None).to_numpy() # mazelist[y,x]
    #mazeList = pd.read_csv("Final_Lab\Final_Lab_maze2.csv", header=None).to_numpy() # mazelist[y,x]
    height, width = mazeList.shape
    mazeList[int(BLOCK_PICKUP_LOC[1]*METERS_TO_MAP),int(BLOCK_PICKUP_LOC[0]*METERS_TO_MAP)] = 3     # mazeList[y,x]
    startLoc = np.array([int(INITIAL_LOC[0]*METERS_TO_MAP),int(INITIAL_LOC[1]*METERS_TO_MAP)])
    # Start interactive plot
    plt.ion()
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111)
    Dijkstra.ExpandWalls(mazeList,padding=3,pad_center=False)
    Dijkstra.Draw_Maze(mazeList,ax)
    # Solve Path
    plt.plot(BLOCK_PLACE_LOC[0]*METERS_TO_MAP,-BLOCK_PLACE_LOC[1]*METERS_TO_MAP,"b")
    plt.plot(BLOCK_PICKUP_LOC[0]*METERS_TO_MAP,-BLOCK_PICKUP_LOC[1]*METERS_TO_MAP,"c")
    plt.pause(1)
    mazeList[int(INITIAL_LOC[1]*METERS_TO_MAP),int(INITIAL_LOC[0]*METERS_TO_MAP)] = 0
    
    # Initialize Robot
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    ep_sensor = ep_robot.sensor
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_sensor.sub_distance(freq=10,callback=sub_distance_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    ep_gripper.open()
    ep_arm.moveto(180,-80).wait_for_completed()
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    
    run_bool=True
    framecount = 1
    path_count=0
    target = "pickup"
    yaw_adjustment=est_heading-initial_head
    #Map testing Zone
    while(not run_bool):
        # print("{}   {}".format(est_heading,yaw_adjustment))
        updateMapLoc()
        print("({:.2f},{:.2f},{:.2f})  maze:({:.2f}, {:.2f})".format(est_x,est_y,est_heading,est_x*METERS_TO_MAP,-est_y*METERS_TO_MAP))
        try:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)  
        except:
            print("Camera Fail")
            continue
        print(frame.shape)
        cv2.imshow("out",frame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        # ep_gripper.open();time.sleep(2);ep_gripper.stop()
        # ep_arm.moveto(180,-80).wait_for_completed()
        # color_range = find_closet_block()
        # pickup_block(color_range)
        # time.sleep(.01)

    if (run_bool):
        lookAround(-50*ANGLE_SIGN,-110*ANGLE_SIGN,num_cycles=1)
        pathDes = Dijkstra.Dijkstra(mazeList, [startLoc[0],startLoc[1],0])
        Dijkstra.PlotPath(pathDes)
        updateMapLoc(update_path=True)
    while(run_bool): 
        updateMapLoc(framecount%25 == 0,framecount%50 == 0)
        if (path_count+2)>len(pathDes):
            updateMapLoc()
            # MOVE TO Final point on path
            while(not odom_pid(pathDes[-1][0]/METERS_TO_MAP,pathDes[-1][1]/METERS_TO_MAP,0 if target=="pickup" else 90*ANGLE_SIGN,tight=True)):
                robo_x_speed, robo_y_speed = responseToSpeed()
                ep_chassis.drive_speed(-1*robo_x_speed,-1*robo_y_speed,1*z_response,timeout=.1)
                time.sleep(.01)
            updateMapLoc()
            if target =="pickup":
                # PICKUP BLOCK
                ep_chassis.drive_speed(0,0,0,timeout=.1)
                color_range = find_closet_block()
                pickup_block(color_range)
                
                # MOVE BACK TO PICKUP LOCATION
                while(not odom_pid(BLOCK_PICKUP_LOC[0],BLOCK_PICKUP_LOC[1],179,tight=False)):
                # while(not odom_pid(est_x,est_y,179,tight=False)):
                    robo_x_speed, robo_y_speed = responseToSpeed()
                    ep_chassis.drive_speed(-1*robo_x_speed,-1*robo_y_speed,1*z_response,timeout=.1)
                    time.sleep(.01)
                    updateMapLoc()
                
                # RESET MAP LOCATIONS AND SOLVE TO PLACE
                updateMapLoc(clear_bool=True)
                lookAround(135*ANGLE_SIGN,170,num_cycles=1)
                mazeList[int(BLOCK_PICKUP_LOC[1]*METERS_TO_MAP),int(BLOCK_PICKUP_LOC[0]*METERS_TO_MAP)] = 0
                mazeList[int(BLOCK_PLACE_LOC[1]*METERS_TO_MAP),int(BLOCK_PLACE_LOC[0]*METERS_TO_MAP)] = 3 
                # startLoc = np.array([int(BLOCK_PICKUP_LOC[1]*METERS_TO_MAP),int(BLOCK_PICKUP_LOC[0]*METERS_TO_MAP)])
                startLoc = np.array([int(est_x*METERS_TO_MAP),int(est_y*METERS_TO_MAP)])
                pathDes = Dijkstra.Dijkstra(mazeList, [startLoc[0],startLoc[1],0])
                path_count=0
                target = "place"
                updateMapLoc(update_path = True)
                # break
            elif target =="place":
                updateMapLoc()
                while(not odom_pid(BLOCK_PLACE_LOC[0],BLOCK_PLACE_LOC[1],90*ANGLE_SIGN,tight=True)):
                    robo_x_speed, robo_y_speed = responseToSpeed()
                    ep_chassis.drive_speed(-1*robo_x_speed,-1*robo_y_speed,1*z_response,timeout=.1)
                    time.sleep(.01)
                    updateMapLoc()
                # ep_arm.moveto(220,15).wait_for_completed()
                act=ep_arm.moveto(220,15);time.sleep(1);act._state='action_succeeded';ep_arm._action_dispatcher._in_progress = {}
                ep_gripper.open();time.sleep(1);ep_gripper.stop()
                print("dropped")
                ep_chassis.drive_speed(-.25,0,0);time.sleep(1)
                ep_chassis.drive_speed(0,0,0,timeout=.1)
                # ep_arm.moveto(180,-80).wait_for_completed()
                act=ep_arm.moveto(180,-80);time.sleep(1);act._state='action_succeeded';ep_arm._action_dispatcher._in_progress = {}
                # RESET MAP LOCATIONS AND SOLVE TO PICKUP
                updateMapLoc(clear_bool=True)
                lookAround(-40*ANGLE_SIGN,-80*ANGLE_SIGN,num_cycles=1)
                mazeList[int(BLOCK_PICKUP_LOC[1]*METERS_TO_MAP),int(BLOCK_PICKUP_LOC[0]*METERS_TO_MAP)] = 3
                mazeList[int(BLOCK_PLACE_LOC[1]*METERS_TO_MAP),int(BLOCK_PLACE_LOC[0]*METERS_TO_MAP)] = 0 
                # startLoc = np.array([int(BLOCK_PLACE_LOC[1]*METERS_TO_MAP),int(BLOCK_PLACE_LOC[0]*METERS_TO_MAP)])
                startLoc = np.array([int(est_x*METERS_TO_MAP),int(est_y*METERS_TO_MAP)])
                pathDes = Dijkstra.Dijkstra(mazeList, [startLoc[0],startLoc[1],0])
                path_count=0
                target="pickup"
                updateMapLoc(update_path= True)
                # break
        # Get desired Heading along path
        tangent_vector = [pathDes[path_count+1][0]/METERS_TO_MAP-pathDes[path_count+0][0]/METERS_TO_MAP,pathDes[path_count+1][1]/METERS_TO_MAP-pathDes[path_count+0][1]/METERS_TO_MAP]
        old_tangent_angle_rad = tangent_angle_rad
        tangent_angle_rad = (-np.arctan2(tangent_vector[1],tangent_vector[0]))
        rounded_tangent = np.rad2deg(tangent_angle_rad+(tangent_angle_rad-old_tangent_angle_rad)/2)
        if abs(rounded_tangent)>180:
            continue
        # FOLLOWS PATH
        if odom_pid(pathDes[path_count][0]/METERS_TO_MAP,pathDes[path_count][1]/METERS_TO_MAP,rounded_tangent):
            path_count+=2
        robo_x_speed, robo_y_speed = responseToSpeed()
        ep_chassis.drive_speed(-1*robo_x_speed,-1*robo_y_speed,1*z_response,timeout=.1)
        
        if (framecount%250 == 0): # Recalc dijkstra's
            plt.cla()
            print("Clear")
            Dijkstra.Draw_Maze(mazeList,ax)
            try:
                pathDes = Dijkstra.Dijkstra(mazeList, [int(pathDes[path_count][0]),int(pathDes[path_count][1]),0])
            except:
                ep_chassis.drive_speed(-.25,0,0);time.sleep(1);ep_chassis.drive_speed(0,0,0,timeout=.1)
                startLoc = np.array([int(est_x*METERS_TO_MAP),int(est_y*METERS_TO_MAP)])
                pathDes = Dijkstra.Dijkstra(mazeList, [startLoc[0],startLoc[1],0])
            path_count=0
            Dijkstra.PlotPath(pathDes)
            updateMapLoc(update_path=True)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(.01)
        framecount += 1

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