'''
CMSC477 - LAB 2
Bradley Dennis, Jack Mirenzi
'''
from ultralytics import YOLO
import cv2
import time
from robomaster import robot
from robomaster import camera
from roboflow import Roboflow
import numpy as np


if __name__ == "__main__":     
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    rf = Roboflow(api_key="kKusTXhj0ObVGmi9slHp")
    project = rf.workspace().project("project2-l7rdy")
    model = project.version(4).model

    cv2.namedWindow("Webcam")
    vid = cv2.VideoCapture(0)
    # cv2.namedWindow("RoboCam")
    # vid = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   

    if vid.isOpened(): # try to get the first frame
        ret, image = vid.read()
    else:
        ret = False

    res_plotted = image

    minWidth = image.shape[1]
    relBlockHeight = 150 # Pixel tall block for desired location
    minFound = True
    atDistance = False

    while ret:
        blockInSight = False
        # MODEL BOUNDING BOXES ON BLOCK
        cv2.imshow("Webcam", res_plotted)
        ret, image = vid.read()

        if image is not None:
            start = time.time()
            preds = model.predict(image, confidence=40, overlap=30).json()['predictions']
            for pred in preds:
                if pred['class'] == 'lego':
                    color = (0,255,255)
                    blockInSight = True
                if pred['class'] == 'robot':
                    color = (255,255,0)
                image = cv2.rectangle(img=image, pt1=(round(pred['x']-pred['width']/2),round(pred['y']-pred['height']/2)),
                                      pt2=(round(pred['x']+pred['width']/2),round(pred['y']+pred['height']/2)),
                                      color=color, thickness=2)
            res_plotted = image
        print("fps: {:.1f}".format(1.0/(time.time()-start)))
        key = cv2.waitKey(10)
        if key == ord('q'):
                break
        
        # if minFound == False:
        #     if blockInSight == False:
        #         print("Searching for block...")
        #         ep_chassis.drive_speed(0, 0, 0.05,timeout=.1)
        #     elif 
        # else:
        #     continue
        #     # move x forward and grab


