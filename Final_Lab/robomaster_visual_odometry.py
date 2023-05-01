# robomaster_visual_odometry.py
# 04-11-2023 First version for CMSC477
# Runs a classical visual odometry algorithm
# based on estimating and decomposing the essential matrix with RANSAC
# The algorithm is adapted from: https://github.com/uoip/monoVO-python

import time
import threading

import cv2
import numpy as np 

# To install skimage run
# pip3 install scikit-image
from skimage.metrics import structural_similarity as ssim

from robomaster import robot

class TwoFrameEssentialMatrixVisualOdometry:
    def __init__(self, K,
                 min_features=1500,
                 max_frames_between_detection=10,
                 min_ransac_feature_ratio=0.25):
        self.min_features = min_features
        self.max_frames_between_detection = max_frames_between_detection
        self.min_ransac_feature_ratio = min_ransac_feature_ratio

        self.lk_winSize = (21, 21)
        self.lk_criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)

        self.K = K
        self.D = np.zeros((4,)) # We assume the passed in frames are undistorted
        
        self.last_features = None
        self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        # self.detector = cv2.SIFT_create(nfeatures=200)

        self.T_wc = np.vstack((np.hstack((np.eye(3), np.zeros((3,1)))),
                               [0, 0, 0, 1]))

    def update(self, frame, last_frame, frame_count, scale):
        if self.last_features is None:
            new_features = self.detector.detect(frame)
            self.last_features = np.array([f.pt for f in new_features], dtype=np.float32)
            return

        # Track features with Lucas-Kanade optical flow with pyramids
        # Throw away features that have been lost as indicated by the status variable
        new_features, status, _ = cv2.calcOpticalFlowPyrLK(last_frame, frame, self.last_features, None,
                                                           winSize  = self.lk_winSize,
                                                           criteria = self.lk_criteria)
        status = status.flatten()
        self.last_features = self.last_features[status == 1]
        new_features       =      new_features [status == 1]

        # Recover the essential matrix from the feature pairs using RANSAC to reject outliers
        E, mask = cv2.findEssentialMat(self.last_features, new_features, self.K, self.D, self.K, self.D,
                                       method=cv2.RANSAC, prob=0.999, threshold=1.0)

        # Using triangulation and the positive depth condition to choose one of the four
        # pairs of rotation and translation determined by the essential matrix
        # Use mask to throw away correspondences that RANSAC determined to be over the threshold
        # e.g. x' E x > threshold
        retval, R_c1c2, t_unit_c1c2, mask = cv2.recoverPose(E, new_features, self.last_features, self.K, mask=mask)
        t_c1c2 = t_unit_c1c2 * scale

        # Compose the transforms to get the new pose of the camera in the pose
        T_c1c2 = np.vstack((np.hstack((R_c1c2, t_c1c2)),
                            [0, 0, 0, 1]))
        self.T_wc = self.T_wc @ T_c1c2

        # Detect new features if any of the following conditions are met
        ransac_ratio = np.sum(mask) / new_features.shape[0]
        if (new_features.shape[0] < self.min_features # Not enough tracked features
            or frame_count % self.max_frames_between_detection == 0 # N frames have gone by
            or ransac_ratio < self.min_ransac_feature_ratio): # A lot of features failed the RANSAC threshold
            new_features = self.detector.detect(frame)
            self.last_features = np.array([f.pt for f in new_features], dtype=np.float32)
        else:
            self.last_features = new_features

def sub_position_handler(p, x_new):
    x_new[0] = p[0]
    x_new[1] = p[1]
    x_new[2] = p[2]
    # print("chassis position: x: {}".format(x_new))

wait_to_start_moving = True
def move_square(ep_chassis, x_len=0.5, y_len=0.5, speed=1.0):
    while wait_to_start_moving: time.sleep(0.1)
    while True:
        ep_chassis.move(x=x_len,  y=0,      z=0, xy_speed=speed).wait_for_completed()
        ep_chassis.move(x=0,      y=y_len,  z=0, xy_speed=speed).wait_for_completed()
        ep_chassis.move(x=-x_len, y=0,      z=0, xy_speed=speed).wait_for_completed()
        ep_chassis.move(x=0,      y=-y_len, z=0, xy_speed=speed).wait_for_completed()

if __name__ == '__main__':
    ssim_threshold = 0.7 # This has a big impact on the number of frames odometry is computed between
    K = np.array([[631.05345607,         0.0, 646.86001960],
                  [         0.0, 633.5803277, 357.86951071],
                  [         0.0,         0.0,          1.0]])
    D = np.array([0.1726052, 0.43400192, -0.43320789, 0.04646433]) # Distortion parameters for fisheye camera
    mapx, mapy = cv2.fisheye.initUndistortRectifyMap(K, D, None, K, (1280,720), cv2.CV_32FC1)

    vo = TwoFrameEssentialMatrixVisualOdometry(K)

    # Wheel odometry position estimates for rescaling visual odometry t vector
    x_list = []
    x_old = np.zeros((3,))
    x_new = np.zeros((3,))
    frame_undistorted_gray_old = None

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_robot.chassis.sub_position(freq=50, callback=lambda p: sub_position_handler(p, x_new))
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)

    x = threading.Thread(target=move_square, daemon=True, args=(ep_robot.chassis,))
    x.start()

    # For visualization
    trajectory_plot = np.zeros((480, 640, 3), dtype=np.uint8)
    plot_off_x = int(trajectory_plot.shape[1]/2)
    plot_off_y = int(trajectory_plot.shape[0]/2)
    # Each pixel in the plot is 1 meter / plot_scale
    plot_scale = 100

    frame_count = 0
    while True:
        # Use "newest" to avoid video corruption
        frame = ep_camera.read_cv2_image(strategy='newest', timeout=5.0)
        frame_count += 1

        frame_undistorted = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

        frame_undistorted_gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)

        if frame_undistorted_gray_old is None:
            frame_undistorted_gray_old = frame_undistorted_gray
            continue

        wait_to_start_moving = False # Make the robot start moving only after we have at least two frames

        if x_old is None:
            x_old = np.copy(x_new)

        # Visual odometry fails when the motion is very small
        # Use Structure Similarity Index Measure (SSIM) as a heuristic
        # to determine when the image has changed "enough"
        # https://en.wikipedia.org/wiki/Structural_similarity
        ssim_const = ssim(frame_undistorted_gray, frame_undistorted_gray_old)
        # print('ssim', ssim_const)
        if ssim_const > ssim_threshold:
            continue

        # The visual odometry translation vector always has unit length
        # so scale it by the amount of translation since the last VO update
        # as measured by the robots odometry
        scale = np.linalg.norm(x_old-x_new)
        vo.update(frame_undistorted_gray, frame_undistorted_gray_old, frame_count, scale)

        p_camera_w = vo.T_wc[0:3, 3]

        # Plot the visual odometry trajectory and the robots odometry trajectory
        cv2.circle(trajectory_plot,
                   (int(plot_scale * p_camera_w[0] + plot_off_x),
                    int(plot_scale * p_camera_w[2] + plot_off_y)),
                   1, (0,255,0), 1)
        cv2.circle(trajectory_plot,
                   (int(plot_scale * x_new[1] + plot_off_x),
                    int(plot_scale * x_new[0] + plot_off_y)),
                   1, (0,0,255), 1)

        # Plot all the tracked features on the image as blue circles
        for i in range(vo.last_features.shape[0]):
            cv2.circle(frame_undistorted,
                       (int(vo.last_features[i, 0]),
                        int(vo.last_features[i, 1])),
                       2, (255, 0, 0), 1)

        cv2.imshow('Camera with features', frame_undistorted)
        cv2.imshow('Trajectory plot', trajectory_plot)
        cv2.waitKey(1)
        #cv2.waitKey(0)

        frame_undistorted_gray_old = frame_undistorted_gray
        x_old = np.copy(x_new)
