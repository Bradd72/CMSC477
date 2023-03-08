from pupil_apriltags import Detector
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from robomaster import robot
from robomaster import camera
import pandas as pd

MAZE_TO_METERS = .0536
METERS_TO_MAZE = 1/MAZE_TO_METERS

robot_coord = [0,0,0]
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

def marker_transform(xcoord,ycoord,deg_angle):
    temp_rot = np.eye(3)*-1
    temp_rot[2,2] = 1
    rot, jaco = cv2.Rodrigues(np.array([ 0,np.deg2rad(deg_angle),0]))
    transform = np.eye(4)
    transform[:3,:3] = np.matmul(temp_rot,rot)
    transform[0,3] = xcoord*MAZE_TO_METERS
    transform[2,3] = ycoord*MAZE_TO_METERS
    transform[1,3] = .15
    return transform

def pose_transform(pose):
    temp_rot = np.eye(3)*-1
    temp_rot[2,2] = 1
    rot, jaco = cv2.Rodrigues(pose[1])
    transform = np.eye(4)
    transform[:3,:3] = np.matmul(rot,temp_rot)
    transform[0,3] = pose[0][0] *1
    transform[1,3] = pose[0][1] *1
    transform[2,3] = pose[0][2] *1
    return np.linalg.inv(transform)

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

            K=np.array([[184.752*1.7, 0, 320], [0, 184.752*1.7, 180], [0, 0, 1]])

            results = at_detector.detect(gray, estimate_tag_pose=False)

            x_coord=[]
            z_coord=[]
            rotation=[]
            weights=[]
            
            for res in results:
                pose = find_pose_from_tag(K, res)
                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                
                if (abs(pose[0][0])>.75):
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                    cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                    continue
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 255, 0), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                T_april_cam = pose_transform(pose)
                
                for i in range(len(marker_List)):
                    if marker_List[i][0] == res.tag_id:
                        T_globe_april = marker_transform(marker_List[i,1],marker_List[i,2],1*(marker_List[i,3]))
                        T_globe_cam = np.matmul(T_globe_april,T_april_cam)
                        rot_ = np.matmul([1,0,0],T_globe_cam[:3,:3])
                        rotation.append(np.arctan2(rot_[2],rot_[0]))
                        x_coord.append(T_globe_cam[0,3])
                        z_coord.append(T_globe_cam[2,3])
                        # weights.append(1)
                        weights.append(abs(T_globe_cam[0,3])**-1)
                        # print("{}: {}   {}   {}".format(res.tag_id,r.as_euler("xyz",degrees=True),np.rad2deg(pose[1]),r.as_euler("xyz",degrees=True)-np.rad2deg(pose[1])))
                        # print("{}: {} {} ({}, {})".format(res.tag_id,pose[0]*METERS_TO_MAZE,np.rad2deg(pose[1]),marker_List[i][1],marker_List[i][2]))
                
                
        
            if x_coord != []:
                robot_coord = [np.average(x_coord,weights=weights),np.average(z_coord,weights=weights),np.average(rotation,weights=weights)]
            # print("{} {} {}".format(res.tag_id,pose[0],pose[1]))
            print("{:.3f},{:.3f}    {:.2f},{:.2f} rot={:.2f}   {:.2f}, {:.2f}, {:.2f} ".format(
                robot_coord[0],robot_coord[1],robot_coord[0]*METERS_TO_MAZE,robot_coord[1]*METERS_TO_MAZE,
                np.rad2deg(robot_coord[2]),np.var(x_coord),np.var(z_coord),np.var(rotation)))

            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)