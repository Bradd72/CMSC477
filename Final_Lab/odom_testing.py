import numpy as np
from robomaster import robot
import time




K_p = .45;K_i = .05;K_d=0;K_p_z = 45;K_i_z = .5;K_d_z = 5
x_error=0;x_integrator=0;x_response=0;est_x=0
y_error=0;y_integrator=0;y_response=0;est_y=0
z_error=0;z_integrator=0;z_response=0;est_heading=0
error_norm=np.ones((50,1))*100;error_tol=.010;error_count=0
prev_time=time.time()
yaw=0; pitch=0; roll=0
robo_x=0;robo_y=0;delta_x=0;delta_y=0

ir_distance=0

def sub_distance_handler(dist_info):
    global ir_distance
    ir_distance = dist_info[0]

def sub_attitude_info_handler(attitude_info):
    global yaw, pitch, roll
    yaw, pitch, roll = attitude_info

def sub_position_handler(position_info):
    global robo_x, robo_y
    robo_x, robo_y, z = position_info
    print("chassis position: x:{0}, y:{1}, z:{2}  x_response:{:.2f},  y_response: {:.2f},  error_norm: {:.2f}, z_response: {:.2f}".format(robo_x, robo_y, z,x_response,y_response,np.mean(error_norm),z_response))

def odom_pid(des_X,des_Y,des_head):
    global y_diff,x_diff,y_integrator,x_integrator,y_error,x_error,y_response,x_response,error_count,error_norm,prev_time
    y_prev=y_error
    x_prev=x_error
    head_error = (est_heading - des_head)
    x_error = (est_y - des_Y)*-1
    y_error = (est_x - des_X)
    time_ = time.time()
    time_step = time_ - prev_time
    prev_time = time_
    if 0<time_step and time_step < .5:
        y_integrator +=y_error*time_step   
        y_diff = (y_error - y_prev) / time_step
        x_integrator +=x_error*time_step  
        x_diff = (x_error - x_prev) / time_step
    else:
        y_integrator = 0; y_diff = 0; x_integrator = 0; x_diff = 0
    y_response = y_error*K_p+y_integrator*K_i+y_diff*K_d
    x_response = x_error*K_p+x_integrator*K_i+x_diff*K_d
    if error_count>=len(error_norm)-1:
        error_count=0
    else:
        error_count+=1

    head_error
    error_norm[error_count]=np.linalg.norm((x_error,y_error,head_error*10))
    if np.mean(error_norm)<error_tol: #
        error_norm=np.ones((50,1))*100
        return True
    else:
        return False


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_sensor.sub_distance(freq=10,callback=sub_distance_handler)

    initial_x=0;initial_y=0;initial_head=0
    path = [(0,.1,0),(0,.2,0)]
    # path = [(0,0,0),(0,.1,0),(.1,.1,0),(.1,0,0),(0,0,0)]
    path_num=0

    run_bool = True
    des_x=10
    des_y=20
    des_head = 0
    i=0
    

    while(path_num<len(path)):
    # while(i<1000000000):
        if odom_pid(path[path_num][0],path[path_num][1],path[path_num][2]):
            path_num+=1
        est_heading=yaw+initial_head
        est_x=robo_x+initial_x
        est_y=robo_y+initial_y
        # print("robo_x: {:.2f}, robo_y: {:.2f}, x: {:.2f}, y: {:.2f}, heading: {:.2f}".format(robo_x,robo_y,est_x,est_y,est_heading))
        print("des_x:{}, des_y:{}, x:{:.2f}, y:{:.2f}, heading:{:.2f},  x_response:{:.2f},  y_response: {:.2f},  error_norm: {:.2f}, z_response: {:.2f}".format(path[path_num][0],path[path_num][1],est_x, est_y,est_heading,x_response,y_response,np.mean(error_norm),z_response))
        # robo_x_speed = x_response*np.cos(est_heading)-y_response*np.sin(est_heading)
        # robo_y_speed = x_response*np.sin(est_heading)-y_response*np.cos(est_heading)
        # robo_x_speed = 0
        # robo_y_speed = .1
        z_response=0
        # ep_chassis.drive_speed(-1*x_response,-1*y_response,z_response,timeout=.1)
        # ep_chassis.drive_speed(.1,.1,0,timeout=.1)
        # i+=1




    ep_chassis.unsub_attitude()
    ep_chassis.unsub_position()
    ep_sensor.unsub_distance()
    

    ep_robot.close()
        
