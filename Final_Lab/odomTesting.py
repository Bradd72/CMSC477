import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import time
import imutils

# np.set_printoptions(formatter={'float': lambda x: "{0:.2f}".format(x)})
run_bool = True
lk_params=dict(winSize  = (21,21), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

if __name__=='__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    ep_gripper.open()
    ep_arm.moveto(180,60).wait_for_completed()

    K = np.array([[631.05345607, 0., 646.8600196 ],[ 0., 633.5803277, 357.86951071], [ 0., 0., 1.]])
    D = np.array([0.1726052, 0.43400192, -0.43320789, 0.04646433])
    focal_length = (K[0,0]+K[1,1])/2
    principle_point = (K[0,2],K[1,2])
    R = np.eye(3)
    t = np.zeros(shape=(3, 3))
    T = np.zeros(shape=(1, 3),dtype='float64')
    num_features=0
    frame_num = 1
    frame_scale = np.ones(shape=(3, 3))*.001

    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    time.sleep(1)
    frame = cv2.undistort(ep_camera.read_cv2_image(strategy="newest", timeout=0.5),K,D)
    time.sleep(.5)
    while run_bool:
        prev_frame = frame
        distorted_frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        frame = cv2.undistort(distorted_frame,K,D)
        if num_features < 2000:
            p0_ = cv2.FastFeatureDetector_create(threshold=50, nonmaxSuppression=True).detect(prev_frame)
            p0 = np.array([x.pt for x in p0_], dtype=np.float32).reshape(-1, 1, 2)
        p1, st, err = cv2.calcOpticalFlowPyrLK(prev_frame, frame,p0, None, **lk_params)
        good_old_pts = p0[st == 1]
        good_new_pts = p1[st == 1]
        E, _ = cv2.findEssentialMat(good_new_pts, good_old_pts, K, cv2.RANSAC, 0.999, 1.0, None)
        # _, R, t, _ = cv2.recoverPose(E, good_old_pts, good_new_pts,K, R.copy(), t.copy(), None)
        _, R_, t, _ = cv2.recoverPose(E, good_old_pts, good_new_pts,K)
        T = T + frame_scale*R.dot(t)
        R = R_.dot(R)
        ## TODO: Add scaling
        num_features = good_new_pts.shape[0]

        frame_num+=1
        print("T:{}".format(np.transpose(T)))
        # print("t:{}  R:{}".format(t,R))

