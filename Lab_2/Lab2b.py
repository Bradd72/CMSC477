import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import time
from roboflow import Roboflow

rf = Roboflow(api_key="kKusTXhj0ObVGmi9slHp")
project = rf.workspace().project("project2-l7rdy")
model = project.version(4).model

pos_x=0
pos_y=0
cX=0
cY=0
K_p = .001
K_i = .0075
K_d = .00005
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
        error_norm=np.ones((50,1))*100
        return True
    else:
        return False

def heading_pid(des_heading):
    global z_diff,z_diff,z_integrator,z_response,error_count_head,error_norm_head,prev_time_head
    z_prev=z_error
    time_ = time.time()
    time_step = time_ - prev_time_head
    prev_time_head = time_
    if time_step < .5:
        z_integrator +=z_error  
        z_diff = (z_error - z_prev) / time_step
    else:
        z_integrator = 0
        z_diff = 0
    z_error = (cX - des_cX)
    z_response = z_error*K_p_z+z_integrator*K_i_z+z_diff*K_d_z
    if error_count_head>=len(error_norm_head)-1:
        error_count_head=0
    else:
        error_count_head+=1
    error_norm_head[error_count_head]=z_error
    if np.mean(error_norm_head)<error_tol: #
        error_norm_head=np.ones((50,1))*100
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
    
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    run_bool=True
    time.sleep(2)

    lower_blue = np.array([90,50,76])
    upper_blue = np.array([135,255,255])
    lower_yel = np.array([20,120,100])
    upper_yel = np.array([50,255,255])
    lower_org = np.array([10,113,180])
    upper_org = np.array([22,255,255])

    des_cX=320
    des_cY=200
    goal="robot"
    while(run_bool):
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
        frame_height, frame_width, c = frame.shape
        output = frame.copy() 
        
        
     

        ## Find Block from Components
        item_found = False
        if goal=='robot':
            # river tracking
            blurred = cv2.medianBlur(frame,9)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            analysis = cv2.connectedComponentsWithStats(mask,4,cv2.CV_32S)
            (totalLabels, label_ids, values, centroid) = analysis  
            x_shift=10
            for i in range(1, totalLabels):
                area = values[i, cv2.CC_STAT_AREA] 
                x = values[i, cv2.CC_STAT_LEFT]
                y = values[i, cv2.CC_STAT_TOP]
                w = values[i, cv2.CC_STAT_WIDTH]
                h = values[i, cv2.CC_STAT_HEIGHT]
                # print("area: {}  w:{} h:{}".format(area,w,h))
                # Checks if Item is big enough and not at the top of screen
                if (area > 1000) and (area < 10000) and (y>60) and (w/h>2):
                    item_found = True
                    componentMask = (label_ids == i).astype("uint8") * 255
                    for j in range(h-1,0,-1):
                        if mask[y+j,x+w-x_shift]:
                            dy2=j
                        if mask[y+j,x+x_shift]:
                            dy1=j
                    cv2.line(output, (x,y+dy1), (x+w, y+dy2), (0, 0, 255),2)
                    (cX_, cY) = centroid[i]
                    cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)
                    heading=np.arctan2(dy2-dy1,w-2*x_shift)
            # Tracking left to right
            fps_time = time.time()
            preds = model.predict(frame, confidence=40, overlap=30).json()['predictions']
            for pred in preds:
                if pred['class'] == 'robot':
                    item_found = True
                    x1=round(pred['x']-pred['width']/2)
                    y1=round(pred['y']-pred['height']/2)
                    x2=round(pred['x']+pred['width']/2)
                    y2=round(pred['y']+pred['height']/2)
                    mask=np.zeros((frame_height, frame_width))
                    mask[x1:x2][y1:y2] = 1
                    cX=pred['x']
                    cY_target=pred['y']
                    cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)
                    cv2.rectangle(img=output, pt1=(x1,y1),pt2=(x2,y2),color=(0,255,255), thickness=2)
            fps = 1.0/(time.time()-fps_time)
        elif goal=='orange':
            blurred = cv2.medianBlur(frame,9)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_org, upper_org)
            analysis = cv2.connectedComponentsWithStats(mask,4,cv2.CV_32S)
            (totalLabels, label_ids, values, centroid) = analysis  
            x_shift=10
            for i in range(1, totalLabels):
                area = values[i, cv2.CC_STAT_AREA] 
                # print("area: {}  w:{} h:{}".format(area,w,h))
                # Checks if Item is big enough and not at the top of screen
                if (area > 1000) and (area < 10000) and (y>60) and (w/h>2):
                    item_found = True
                    componentMask = (label_ids == i).astype("uint8") * 255
                    for j in range(h-1,0,-1):
                        if mask[y+j,x+w-x_shift]:
                            dy2=j
                        if mask[y+j,x+x_shift]:
                            dy1=j
                    cv2.line(output, (x,y+dy1), (x+w, y+dy2), (0, 0, 255),2)
                    (cX, cY) = centroid[i]
                    cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)

        ## Arm Controller
        # print("Robotic Arm: pos x:{0}, pos y:{1}".format(pos_x, pos_y))
        if item_found:
            if goal=='robot':
                # TODO: perfect this part
                if centroid_pid(des_cX=320,des_cY=200) and heading_pid(0) and cY_target>200:
                    ep_chassis.drive_speed(0,0,0)
                    ep_gripper.open()
                    # time.sleep(1)
                    ep_chassis.drive_speed(.3,0,0)
                    time.sleep(.8)
                    ep_chassis.drive_speed(0,0,0)
                    ep_gripper.close()
                    time.sleep(1)
                    ep_chassis.drive_speed(-.5,0,0)
                    time.sleep(1)
                    goal=='orange'
            elif goal=='orange':
                    if centroid_pid(200,300):
                        ep_chassis.drive_speed(.3,-.3,0)
                        time.sleep(.85)
                        ep_chassis.drive_speed(0,0,0)
                        ep_gripper.open()
                        break
            ep_chassis.drive_speed(x_response, y_response,z_response,timeout=.5)
            print("({}, {}) x_response:{:.2f},  y_response: {:.2f},  error_norm: {:.2f},  z_response: {:.2f}, fps: {:.1f}".format(des_cX,des_cY,x_response,y_response,np.mean(error_norm),z_response,fps))
        else:
            ep_chassis.drive_speed(0,0,10,timeout=.5)
            print(goal)
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
    print ('Exiting')
    time.sleep(1)
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
    exit(1)