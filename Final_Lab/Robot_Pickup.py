'''
CMSC477 - Final Lab
Robot responsible for picking and transfering blocks over river
'''
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import time
from roboflow import Roboflow

rf = Roboflow(api_key="kKusTXhj0ObVGmi9slHp")
project = rf.workspace().project("project2-l7rdy")
model = project.version(4).model

def sub_data_handler(sub_info):
    global pos_x, pos_y 
    pos_x, pos_y = sub_info

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis, ep_camera = ep_robot.camera, ep_gripper = ep_robot.gripper, ep_arm = ep_robot.robotic_arm
    
    ep_gripper.open()
    ep_arm.moveto(180,-20).wait_for_completed()
    
    ep_arm.sub_position(freq=5, callback=sub_data_handler)
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    run_bool=True
    
    while(run_bool):
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
        frame_height, frame_width, c = frame.shape
        output = frame.copy() 

        cv2.imshow("out",output)
        time.sleep(1)
        '''
        - Locate robot in the world and pickup location
            - Rotate robot until it sees both the visual markers and pickup zone
       *    - requires visual odometry calculations from camera
                    - Gets robots location in the world
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

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # Destroys all of the windows and closes camera  
    print ('Exiting')
    time.sleep(1)
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
    exit(1)