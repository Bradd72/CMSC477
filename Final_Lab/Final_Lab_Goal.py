import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import time
from roboflow import Roboflow


if __name__=='__main__':

    """
    status = calibrate,ready, placing, stealing

    functions:
        - odomCali: run at the river; sets origin and uses river lenght for scaling
        - odomUpdate: uses visual odometry to find pos change; prev_frame,frame,ignore_box -> x and y delta 
        - odomPID: desX,desY,currX,currY -> x_response,y_response
        - roboflowRobot: frame -> bounding box of robot
        - blockHandoff: 

    - Move to river for odomCali()
    - Main Loop (updateOdom,odomPID,roboflowRobot)
        - Move back to waiting posistion 
        - With trigger from other bot move to Pickup Location [defined coordinate]
            - Grab block and then send success trigger
        - With too long of wait go to steal
        - Move to Goal
            -
    """
