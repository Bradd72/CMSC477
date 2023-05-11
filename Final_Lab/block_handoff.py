import numpy as np
import cv2
from robomaster import robot
from robomaster import camera
import time

K_p_pix=.0005; K_i_pix=.000002; K_d_pix=0; cX=0; cY=0
error_norm_pix=np.ones((20,1))*100;error_tol_pix=15
y_pix_diff=0;x_pix_diff=0;y_pix_integrator=0;x_pix_integrator=0;y_pix_error=0;x_pix_error=0;y_response=0;x_response=0;error_count_pix=0;prev_time=time.time()

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
            ep_arm.move(0,50).wait_for_completed()
            break

        ep_chassis.drive_speed(x_response*-1, y_response*1,0,timeout=.5)


        cv2.imshow("out",output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def block_handoff():
    global cX,cY
    # blue_range = (np.array([108,61,111]),np.array([123,255,255]),"blue")
    blue_range = (np.array([100,140,40]),np.array([135,255,255]),"blue")
    run_bool = True
    while(run_bool):
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
        frame_height, frame_width, c = frame.shape
        blurred = cv2.medianBlur(frame,9)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, blue_range[0], blue_range[1])
        output = blurred.copy() 
        analysis = cv2.connectedComponentsWithStats(mask,4,cv2.CV_32S)
        (totalLabels, label_ids, values, centroid) = analysis  
        for i in range(1, totalLabels):
            w = values[i, cv2.CC_STAT_WIDTH]
            h = values[i, cv2.CC_STAT_HEIGHT]
            area = values[i, cv2.CC_STAT_AREA] 
            # print("a:{:.2f}  w:{:.2f}  h:{:.2f}".format(area,w,h))
            if (area > 2000) and (area < 10000) and (5<w/h):
                (cX, cY) = centroid[i]
                cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)
        
        if centroid_pid(210,250):
            ep_chassis.drive_speed(.25,0,0,timeout=2.75);time.sleep(2.75)
            ep_chassis.drive_speed(0,0,0);time.sleep(.1)
            ep_gripper.open();time.sleep(2);ep_gripper.stop()
            ep_arm.move(0,50).wait_for_completed()
            break
        
        print("({:.2f},{:.2f})".format(cX,cY))
        ep_chassis.drive_speed(x_response*-1, y_response*1,0,timeout=.5)
        res = cv2.bitwise_and(frame,frame, mask= mask)
        cv2.imshow("res",res)       
        cv2.imshow("out",output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        



if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    ep_gripper.open();time.sleep(.5);ep_gripper.stop()
    ep_arm.moveto(180,-80+140).wait_for_completed()
    ep_gripper.close();time.sleep(.5);ep_gripper.stop()
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    block_handoff()

    print ('Exiting')
    time.sleep(1)
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
    exit(1)