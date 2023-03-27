from ultralytics import YOLO
import cv2
import time
from robomaster import robot
from robomaster import camera
from roboflow import Roboflow

rf = Roboflow(api_key="kKusTXhj0ObVGmi9slHp")
project = rf.workspace().project("project2-l7rdy")
model = project.version(4).model

cv2.namedWindow("Webcam")
# image = cv2.imread("Lab_2\Robot.png")
vid = cv2.VideoCapture(0)

if __name__ == "__main__":
    
    if vid.isOpened(): # try to get the first frame
        ret, image = vid.read()
    else:
        ret = False

    res_plotted = image

    while ret:
        cv2.imshow("Webcam", res_plotted)
        ret, image = vid.read()

        if image is not None:
            start = time.time()
            preds = model.predict(image, confidence=40, overlap=30).json()['predictions']
            for pred in preds:
                image = cv2.rectangle(img=image, pt1=(round(pred['x']-pred['width']/2),round(pred['y']-pred['height']/2)),
                                      pt2=(round(pred['x']+pred['width']/2),round(pred['y']+pred['height']/2)),
                                      color=(0,255,255), thickness=2)
            res_plotted = image
        print("fps: {:.1f}".format(1.0/(time.time()-start)))
        key = cv2.waitKey(50)
        if key == ord('q'):
                break