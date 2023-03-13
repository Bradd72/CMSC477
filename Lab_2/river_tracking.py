import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import time



if __name__ == '__main__':


    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    K_p = 20
    K_i = 10
    K_d = 5
    heading=0
    des_heading=0
    z_error=0
    z_integrator=0
    prev_time=time.time()

    while(1):
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
        frame_height, frame_width, c = frame.shape

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
            print("area: {}  i:{} ".format(area,i))
            if (area > 2500) and (area < 20000):
                componentMask = (label_ids == i).astype("uint8") * 255
                x = values[i, cv2.CC_STAT_LEFT]
                y = values[i, cv2.CC_STAT_TOP]
                w = values[i, cv2.CC_STAT_WIDTH]
                h = values[i, cv2.CC_STAT_HEIGHT]
                for j in range(h-1,0,-1):
                    if mask[y+j,x+w-x_shift]:
                        dy2=j
                    if mask[y+j,x+x_shift]:
                        dy1=j
                cv2.line(output, (x,y+dy1), (x+w, y+dy2), (0, 0, 255),2)
                (cX, cY) = centroid[i]
                cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)
                heading=np.arctan2(dy2-dy1,w-2*x_shift)

        ## Movement Controller
        z_prev=z_error
        time_ = time.time()
        time_step = time_ - prev_time
        if time_step < .5:
            z_integrator +=z_error  
            z_diff = (z_error - z_prev) / time_step
        else:
            z_integrator = 0
            z_diff = 0
        z_error = heading - des_heading
        z_response = z_error*K_p+z_integrator*K_i+z_diff*K_d
        ep_chassis.drive_speed(0, 0, z_response,timeout=.1)
        # print("heading:{:.2f},  z_response: {:.2f}".format(heading,z_response))

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