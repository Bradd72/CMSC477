import numpy as np
import cv2
from robomaster import robot
from robomaster import camera
import time

K_p_pix=.0005; K_i_pix=.0002; K_d_pix=0; cX=0; cY=0
error_norm_pix=np.ones((20,1))*100;error_tol_pix=20
y_pix_diff=0;x_pix_diff=0;y_pix_integrator=0;x_pix_integrator=0;y_pix_error=0;x_pix_error=0;y_response=0;x_response=0;error_count_pix=0;error_norm_pix=0;prev_time=time.time()

def centroid_pid(des_cX,des_cY):
    global y_pix_diff,x_pix_diff,y_pix_integrator,x_pix_integrator,y_pix_error,x_pix_error,y_response,x_response,error_count_pix,error_norm_pix, prev_time
    y_pix_prev=y_pix_error
    x_pix_prev=x_pix_error
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
    y_pix_error = (cX - des_cX)
    y_response = y_pix_error*K_p_pix+y_pix_integrator*K_i_pix+y_pix_diff*K_d_pix
    x_error = (cY - des_cY)*-1
    x_response = x_pix_error*K_p_pix+x_pix_integrator*K_i_pix+x_pix_diff*K_d_pix
    if error_count_pix>=len(error_norm_pix)-1:
        error_count_pix=0
    else:
        error_count_pix+=1
    error_norm_pix[error_count_pix]=np.linalg.norm((x_error,y_pix_error))
    if np.mean(error_norm_pix)<error_tol_pix: #
        error_norm_pix=np.ones((20,1))*100;y_pix_integrator = 0;y_pix_diff = 0;x_integrator = 0;x_diff = 0
        return True
    else:
        return False

def find_closet_block():
    yel_range = (np.array([20,120,100]),np.array([50,255,255]))
    org_range = (np.array([10,113,180]),np.array([22,255,255]))
    item_found = False
    greatest_y=0;best_range=yel_range
    for color_range in [yel_range,org_range]:
        for iter_num in range(50):
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
                    if (area > 750) and (area < 50000):
                        item_found = True
                        componentMask = (label_ids == i).astype("uint8") * 255
                        if centroid[i][1] > greatest_y:
                            greatest_y = centroid[i][1]
                            best_range = color_range
                        cv2.circle(frame, (int(centroid[i][0]), int(centroid[i][1])), 4, (0, 0, 255), -1)
            # cv2.imshow("out",output)
    # print(best_range,greatest_y)
    if item_found:
        return best_range
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
    color_range = find_closet_block()

    pickup_block(color_range)

    print ('Exiting')
    time.sleep(1)
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
    exit(1)