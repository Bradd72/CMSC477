from pupil_apriltags import Detector
import cv2
import numpy as np
import time
from robomaster import robot
from robomaster import camera
import pandas as pd

MAZE_TO_METERS = .0536
METERS_TO_MAZE = 1/MAZE_TO_METERS

robot_coord = [0,0]
marker_List = pd.read_csv("Lab_1\WallLookUp.csv", header=None).to_numpy()

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

def update_pos(dectect_results):
    global robot_coord

def marker_transform(xcoord,ycoord,angle):
    rot, jaco = cv2.Rodrigues(np.array([-.5*np.pi,0,angle]))
    transform = np.eye(4)
    transform[:3,:3] = rot
    transform[0,3] = xcoord*MAZE_TO_METERS
    transform[1,3] = ycoord*MAZE_TO_METERS
    transform[2,3] = .15
    return transform

def pose_transform(pose):
    rot, jaco = cv2.Rodrigues(pose[1])
    transform = np.eye(4)
    transform[:3,:3] = rot
    transform[0,3] = pose[0][0]
    transform[1,3] = pose[0][1]
    transform[2,3] = pose[0][2]
    return transform

def find_pose_from_tag(K, detection):
    m_half_size = tag_size / 2

    marker_center = np.array((0, 0, 0))
    marker_points = []
    marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
    marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
    _marker_points = np.array(marker_points)

    object_points = _marker_points
    image_points = detection.corners

    pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if pnp_ret[0] == False:
        raise Exception('Error solving PnP')

    r = pnp_ret[1]
    p = pnp_ret[2]

    return p.reshape((3,)), r.reshape((3,))

if __name__ == '__main__':
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    tag_size=0.16 # tag size in meters

    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)   
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)

            K=np.array([[184.752, 0, 320], [0, 184.752, 180], [0, 0, 1]])

            results = at_detector.detect(gray, estimate_tag_pose=False)

            x_coord=[]
            y_coord=[]
            for res in results:
                pose = find_pose_from_tag(K, res)
                bot_to_tag_trans = pose_transform(pose)
                
                for i in range(len(marker_List)):
                    if marker_List[i][0] == res.tag_id:
                        globe_to_tag_trans = marker_transform(marker_List[i,1],marker_List[i,2],1*marker_List[i,3])
                        # globe_to_bot_trans = globe_to_tag_trans*bot_to_tag_trans
                        globe_to_bot_trans = bot_to_tag_trans*globe_to_tag_trans
                        # print(globe_to_bot_trans)
                        x_coord.append(globe_to_bot_trans[0,3])
                        y_coord.append(globe_to_bot_trans[1,3])
                        # yaw = pose[1][1] - np.deg2rad()
                        # x_coord.append(marker_List[i][1]-(pose[0][0]*np.sin(yaw)+pose[0][1]*np.cos(yaw))*METERS_TO_MAZE)
                        # y_coord.append(marker_List[i][2]-(pose[0][1]*np.sin(yaw)+pose[0][2]*np.cos(yaw))*METERS_TO_MAZE)

                        # print("{}: {} {} ({}, {})".format(res.tag_id,pose[0]*METERS_TO_MAZE,np.rad2deg(pose[1]),marker_List[i][1],marker_List[i][2]))
                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                
        
            robot_coord = [np.mean(x_coord),np.mean(y_coord)]
            # TODO: figure out why the samples give such bad estimation
            print("{:.3f},{:.3f}    {:.2f},{:.2f}   {:.2f} {:.2f}".format(robot_coord[0],robot_coord[1],robot_coord[0]*METERS_TO_MAZE,robot_coord[1]*METERS_TO_MAZE,np.var(x_coord),np.var(y_coord)))

            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)