from ultralytics import YOLO
import cv2
import time
from robomaster import robot
from robomaster import camera
from roboflow import Roboflow
import numpy as np

pos_x=0
pos_y=0
cX=0
cY=0
K_p = .0001
K_i = .000075
# K_d = .00005
# K_i = 0
K_d = 0
K_p_z = .1
K_i_z = .75
K_d_z = .005
heading=0
x_error=0
x_integrator=0
y_error=0
y_integrator=0
z_error=0
z_integrator=0
error_norm=np.ones((50,1))*100
error_tol=35
error_count=0
error_norm_head=np.ones((50,1))*100
error_count_head=0
prev_time=time.time()
prev_time_head=time.time()

def centroid_pid(des_cX,des_cY):
    global y_diff,x_diff,y_integrator,x_integrator,y_error,x_error,y_response,x_response,error_count,error_norm,prev_time
    y_prev=y_error
    x_prev=x_error
    time_ = time.time()
    time_step = time_ - prev_time
    prev_time = time_
    if time_step < .25:
        y_integrator +=y_error  
        y_diff = (y_error - y_prev) / time_step
        x_integrator +=x_error  
        x_diff = (x_error - x_prev) / time_step
    else:
        y_integrator = 0
        y_diff = 0
        x_integrator = 0
        x_diff = 0
    y_error = (cX - des_cX)
    y_response = y_error*K_p+y_integrator*K_i+y_diff*K_d
    x_error = (cY - des_cY)*-1
    x_response = x_error*K_p+x_integrator*K_i+x_diff*K_d
    if error_count>=len(error_norm)-1:
        error_count=0
    else:
        error_count+=1
    error_norm[error_count]=np.linalg.norm((x_error,y_error))
    if np.mean(error_norm)<error_tol: #
        error_norm=np.ones((50,1))*100
        return True
    else:
        return False

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_gripper.open()
    time.sleep(1)
    ep_arm.moveto(200,0).wait_for_completed()
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    rf = Roboflow(api_key="kKusTXhj0ObVGmi9slHp")
    project = rf.workspace().project("project2-l7rdy")
    model = project.version(4).model

    frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
    frame_height, frame_width, c = frame.shape

    res_plotted = frame

    lower_blue = np.array([90,50,76])
    upper_blue = np.array([135,255,255])
    lower_yel = np.array([20,120,100])
    upper_yel = np.array([50,255,255])
    lower_org = np.array([10,113,180])
    upper_org = np.array([22,255,255])

    des_cX=320
    des_cY=200

    K_pz = 45
    K_iz = 10
    K_dz = 1
    K_px = .0015
    K_ix = 0.002
    K_dx = 0.001
    heading=1
    des_heading=0
    z_error=0
    z_integrator=0
    x_error = 0
    x_integrator = 0
    prev_time=time.time()

    robot_center = frame_width/2

    theta_ave=0; rho_ave=0
    status = 'river align'
    # status = 'ready to drop off'
    x_response=0
    y_response=0
    z_response=0

    while(1):
        # print(status)
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
        frame_height, frame_width, c = frame.shape

        final_frame = frame.copy()
        if status != 'ready to drop off':
            if frame is not None:
                preds = model.predict(frame, confidence=40, overlap=30).json()['predictions']
                for pred in preds:
                    if pred['class'] == 'lego':
                        color = (0,255,255)
                        blockInSight = True
                    if pred['class'] == 'robot':
                        color = (255,255,0)
                        robot_center = pred['x']
                    cv2.rectangle(img=final_frame, pt1=(round(pred['x']-pred['width']/2),round(pred['y']-pred['height']/2)),
                                        pt2=(round(pred['x']+pred['width']/2),round(pred['y']+pred['height']/2)),
                                        color=color, thickness=2)
                    cv2.circle(final_frame, center=(round(pred['x']), round(pred['y'])), radius=4, color=color, thickness=-1)

        if status == 'river align':
            ## Image Processing
            blurred = cv2.medianBlur(frame,9)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([90,50,76])
            upper_blue = np.array([135,255,255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            output = blurred.copy() 
            analysis = cv2.connectedComponentsWithStats(mask,4,cv2.CV_32S)
            (totalLabels, label_ids, values, centroid) = analysis  

            ## Find Angle from Components
            x_shift=10
            for i in range(1, totalLabels):
                area = values[i, cv2.CC_STAT_AREA] 
                x = values[i, cv2.CC_STAT_LEFT]
                y = values[i, cv2.CC_STAT_TOP]
                w = values[i, cv2.CC_STAT_WIDTH]
                h = values[i, cv2.CC_STAT_HEIGHT]
                # print("area: {}  i:{} ".format(area,i))
                if (area > 1000) and (area < 30000) and (y>60):
                    componentMask = (label_ids == i).astype("uint8") * 255

                    for j in range(h-1,0,-1):
                        if mask[y+j,x+w-x_shift]:
                            dy2=j
                        if mask[y+j,x+x_shift]:
                            dy1=j
                    cv2.line(final_frame, (x,y+dy1), (x+w, y+dy2), (0, 0, 255),2)
                    (cX, cY) = centroid[i]
                    cv2.circle(final_frame, (int(cX), int(cY)), 4, (0, 0, 255), -1)
                    heading=np.arctan2(dy2-dy1,w-2*x_shift)

            ## Movement Controller
            z_prev=z_error
            time_ = time.time()
            time_step = time_ - prev_time
            z_error = heading - des_heading
            if time_step < .75 and time_step != 0:
                z_integrator += z_error*time_step
                z_diff = (z_error - z_prev) / time_step
            else:
                z_integrator = 0
                z_diff = 0
            z_response = z_error*K_pz+z_integrator*K_iz+z_diff*K_dz
            if np.abs(heading) <= 0.03:
                ep_chassis.drive_speed(0, 0, 0,timeout=.1)
                status = 'center robot'
            else:
                ep_chassis.drive_speed(0, 0, z_response,timeout=.1)
                print("heading:{:.2f},  z_response: {:.2f}".format(heading,z_response))
            prev_time = time_    

        if status == 'center robot':
            ## Movement Controller
            x_prev=x_error
            time_ = time.time()
            time_step = time_ - prev_time
            x_error = robot_center - frame_width/2
            if time_step < .75 and time_step != 0:
                x_integrator +=x_error*time_step
                x_diff = (x_error - x_prev) / time_step
            else:
                x_integrator = 0
                x_diff = 0
            x_response = x_error*K_px + x_integrator*K_ix + x_diff*K_dx
            if np.abs(x_error) <= 20:
                ep_chassis.drive_speed(0, 0, 0,timeout=.1)
                status = 'move to river'
            else:
                ep_chassis.drive_speed(0, x_response, 0,timeout=.1)
                print("x offset:{:.2f},  x_response:{:.2f}".format(x_error,x_response))
            prev_time = time_

        if status == 'move to river':
            ## Image Processing
            blurred = cv2.medianBlur(frame,9)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([105,50,50])
            upper_blue = np.array([135,255,255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            output = blurred.copy() 
            analysis = cv2.connectedComponentsWithStats(mask,4,cv2.CV_32S)
            (totalLabels, label_ids, values, centroid) = analysis  

            ## Find Angle from Components
            x_shift=10
            for i in range(1, totalLabels):
                area = values[i, cv2.CC_STAT_AREA] 
                x = values[i, cv2.CC_STAT_LEFT]
                y = values[i, cv2.CC_STAT_TOP]
                w = values[i, cv2.CC_STAT_WIDTH]
                h = values[i, cv2.CC_STAT_HEIGHT]
                # print("area: {}  i:{} ".format(area,i))
                if (area > 1000) and (area < 30000) and (y>60):
                    componentMask = (label_ids == i).astype("uint8") * 255

                    for j in range(h-1,0,-1):
                        if mask[y+j,x+w-x_shift]:
                            dy2=j
                        if mask[y+j,x+x_shift]:
                            dy1=j
                    cv2.line(final_frame, (x,y+dy1), (x+w, y+dy2), (0, 0, 255),2)
                    (cX, cY) = centroid[i]
                    cv2.circle(final_frame, (int(cX), int(cY)), 4, (0, 0, 255), -1)
                    heading=np.arctan2(dy2-dy1,w-2*x_shift)
            
            ## Movement Controller
            x_prev=x_error
            time_ = time.time()
            time_step = time_ - prev_time
            x_error = (0.7*frame_height) - cY
            if time_step < .75 and time_step != 0:
                x_integrator +=x_error*time_step
                x_diff = (x_error - x_prev) / time_step
            else:
                x_integrator = 0
                x_diff = 0
            x_response = x_error*K_px + x_integrator*K_ix + x_diff*K_dx
            if np.abs(x_error) <= 15:
                ep_chassis.drive_speed(.15,0,0)
                time.sleep(.8)
                ep_chassis.drive_speed(0, 0, 0,timeout=.1)
                #ep_arm.moveto(250,0).wait_for_completed()
                ep_gripper.close()
                time.sleep(5)
                ep_chassis.drive_speed(-.15,0,.2)
                time.sleep(.75)
                ep_chassis.drive_speed(0,0,0)
                status = 'ready to drop off'
                print("ready to drop off")
                x_response=0
                y_response=0
                z_response=0
                y_integrator = 0
                y_diff = 0
                x_integrator = 0
                x_diff = 0
            else:
                ep_chassis.drive_speed(x_response, 0, 0,timeout=.1)
                print("y offset:{:.2f},  x_response:{:.2f}, area:{:.2f}".format(x_error,x_response,area))
            prev_time = time_

        if status == 'ready to drop off':
            # print("ready to drop off")
            item_found = False
            blurred = cv2.medianBlur(frame,9)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_org, upper_org)
            analysis = cv2.connectedComponentsWithStats(mask,4,cv2.CV_32S)
            (totalLabels, label_ids, values, centroid) = analysis  
            x_shift=10
            for i in range(1, totalLabels):
                w = values[i, cv2.CC_STAT_WIDTH]
                h = values[i, cv2.CC_STAT_HEIGHT]
                area = values[i, cv2.CC_STAT_AREA] 
                # print("area: {}  w:{} h:{}  {}".format(area,w,h,w/h))
                # Checks if Item is big enough and if looking for orange will make sure it is wider than tall
                if (area > 1000) and (area < 10000)  and w/h>1:
                    item_found = True
                    componentMask = (label_ids == i).astype("uint8") * 255
                    (cX, cY) = centroid[i]
                    cv2.circle(final_frame, (int(cX), int(cY)), 4, (0, 0, 255), -1)
            if item_found:
                if centroid_pid(200,300):
                    ep_chassis.drive_speed(.3,-.3,0)
                    time.sleep(.85)
                    ep_chassis.drive_speed(0,0,0)
                    ep_gripper.open()
                    break
                print("({}, {}) x_response:{:.2f},  y_response: {:.2f},  error_norm: {:.2f}".format(cX,cY,x_response,y_response,np.mean(error_norm),z_response))
                ep_chassis.drive_speed(x_response, y_response,0,timeout=.5)
            else:
                print("looking")
                ep_chassis.drive_speed(0,0,10,timeout=.1)

        ## OpenCV Windows
        res = cv2.bitwise_and(frame,frame, mask=mask)
        cv2.imshow('frame',final_frame)
        # cv2.imshow('mask',mask)
        cv2.imshow('res',res)
        # cv2.imshow("out",output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Destroys all of the windows and closes camera  
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
    print ('Exiting')
    exit(1)