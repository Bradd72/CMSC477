# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from pupil_apriltags import Detector
import cv2
import numpy as np
import time
from robomaster import robot
from robomaster import camera

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

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
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    x_offset = .35
    y_offset = 0

    K_p = 1.5
    K_i = .15
    K_d = .1

    x_val=0
    y_val=0
    prog_time=time.time()
    time_=prog_time

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
            
            for res in results:
                pose = find_pose_from_tag(K, res)
                rot, jaco = cv2.Rodrigues(pose[1], pose[1])
                # print(rot)

                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)

                # print(pose)
                
                x_prev = x_val
                y_prev = y_val
                prev_time = time_
                x_val =  pose[0][2] - x_offset
                y_val =  pose[0][0] - y_offset
                time_ = time.time()
                time_step = time_ - prev_time
                if time_step < .5:
                    x_integrator+=x_val
                    y_integrator+=y_val
                    x_diff = (x_val - x_prev) / time_step
                    y_diff = (y_val - y_prev) / time_step
                else:
                    x_integrator = 0
                    y_integrator = 0
                    x_diff = 0
                    y_diff = 0
                # if abs(x_integrator)>2:
                #     x_integrator-=x_val
                # if abs(y_integrator)>2:
                #     y_integrator-=y_val
                angle =  np.rad2deg(np.arctan2(y_val,x_val))
                if abs(angle)>15 and x_val>.2:
                    z_val = angle
                    y_val = 0
                else:
                    z_val=0
                x_response = x_val*K_p + x_diff*K_d + x_integrator*K_i
                y_response = y_val*K_p + y_diff*K_d + y_integrator*K_i
                z_response = z_val*K_p
                ep_chassis.drive_speed(x_response, y_response, z_response,timeout=.1)
                print("pos: {},{} integrator {},{}  diff:{},{}".format(x_val*K_p,y_val*K_p,x_integrator*K_i,y_integrator*K_i,x_diff*K_d,y_diff*K_d))

            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)




