import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import time

pos_x=0
pos_y=0
cX=0
cY=0
K_p = .001
K_i = .005
K_d = .00005
heading=0
x_error=0
x_integrator=0
y_error=0
y_integrator=0
error_norm=np.ones((100,1))*1000
error_tol=30
error_count=0
prev_time=time.time()

def sub_data_handler(sub_info):
    global pos_x, pos_y 
    pos_x, pos_y = sub_info

def centroid_pid(des_cX,des_cY):
    global y_diff,x_diff,y_integrator,x_integrator,y_error,x_error,y_response,x_response,error_count,error_norm
    y_prev=y_error
    x_prev=x_error
    time_ = time.time()
    time_step = time_ - prev_time
    if time_step < .5:
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
        return True
    else:
        return False

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    ep_gripper.open()
    ep_arm.moveto(180,-20).wait_for_completed()
    
    ep_arm.sub_position(freq=5, callback=sub_data_handler)
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    run_bool=True
    time.sleep(2)

    lower_yel = np.array([12,120,100])
    upper_yel = np.array([50,255,255])
    lower_org = np.array([10,113,180])
    upper_org = np.array([30,255,255])

    des_cX=320
    des_cY=200
    theta_ave=0; rho_ave=0
    item_found = False
    goal="yellow"
    while(run_bool):
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
        frame_height, frame_width, c = frame.shape

        ## Image Processing
        blurred = cv2.medianBlur(frame,9)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        if goal=='yellow':
            mask = cv2.inRange(hsv, lower_yel, upper_yel)
        elif goal=='orange':
            mask = cv2.inRange(hsv, lower_org, upper_org)
        output = blurred.copy() 
        analysis = cv2.connectedComponentsWithStats(mask,4,cv2.CV_32S)
        (totalLabels, label_ids, values, centroid) = analysis  

        ## Find Block from Components
        x_shift=10
        for i in range(1, totalLabels):
            w = values[i, cv2.CC_STAT_WIDTH]
            h = values[i, cv2.CC_STAT_HEIGHT]
            area = values[i, cv2.CC_STAT_AREA] 
            # print("area: {}  i:{} ".format(area,i))
            item_found = False
            # Checks if Item is big enough and if looking for orange will make sure it is wider than tall
            if (area > 2000) and (area < 10000) and (goal=='yellow' or (goal=='orange' and w/h>1)):
                item_found = True
                componentMask = (label_ids == i).astype("uint8") * 255
                (cX, cY) = centroid[i]
                cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)

        ## Arm Controller
        # print("Robotic Arm: pos x:{0}, pos y:{1}".format(pos_x, pos_y))
        if item_found:
            if centroid_pid(des_cX=des_cX,des_cY=des_cY):
                if goal=='yellow':
                    if des_cY == 200:
                        des_cY=300
                    else:
                        ep_chassis.drive_speed(0,0,0)
                        time.sleep(.1)
                        ep_gripper.close()
                        time.sleep(2)
                        ep_gripper.stop()
                        ep_arm.move(0,50)    
                        goal=='orange'
                elif goal=='orange':
                    
            ep_chassis.drive_speed(x_response, y_response,0,timeout=.5)
        else:
            ep_chassis.drive_speed(0,0,1,timeout=.5)
        print("x_response:{:.2f},  y_response: {:.2f},  error_norm: {:.2f}".format(x_response,y_response,np.mean(error_norm)))
        # print("cX:{:.2f},  cY: {:.2f}".format(cX,cY))
        ## OpenCV Windows
        res = cv2.bitwise_and(frame,frame, mask= mask)
        # cv2.imshow('frame',frame)
        # cv2.imshow('mask',mask)
        cv2.imshow('res',res)
        cv2.imshow("out",output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Destroys all of the windows and closes camera  
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
    print ('Exiting')
    exit(1)