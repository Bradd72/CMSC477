import numpy as np
from robomaster import robot
import time




K_p = .35;K_i = .25;K_d=0.1;K_p_z = 1;K_i_z = .5;K_d_z = 0
x_error=0;x_diff=0;x_integrator=0;x_response=0;est_x=0
y_error=0;y_diff=0;y_integrator=0;y_response=0;est_y=0
head_error=0;head_diff=0;head_integrator=0;z_response=0;est_heading=0
error_norm=np.ones((50,1))*100;error_tol=.025;error_count=0
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
    global y_diff,x_diff,head_diff,y_integrator,x_integrator,head_integrator,y_error,x_error,head_error,y_response,x_response,z_response,error_count,error_norm,prev_time
    y_prev=y_error
    x_prev=x_error
    head_prev=head_error
    head_error = (est_heading - des_head)
    # x_error = (est_y - des_Y)*-1
    # y_error = (est_x - des_X)
    y_error = (est_y - des_Y)*1
    x_error = (est_x - des_X)*1
    time_ = time.time()
    time_step = time_ - prev_time
    prev_time = time_
    if 0<time_step and time_step < .5:
        y_integrator +=y_error*time_step   
        y_diff = (y_error - y_prev) / time_step
        x_integrator +=x_error*time_step  
        x_diff = (x_error - x_prev) / time_step
        head_integrator +=head_error*time_step  
        head_diff = (head_error - head_prev) / time_step
        
    else:
        print("long wait")
        y_integrator = 0; y_diff = 0; x_integrator = 0; x_diff = 0; head_integrator=0; head_diff=0
    y_response = y_error*K_p+y_integrator*K_i+y_diff*K_d
    x_response = x_error*K_p+x_integrator*K_i+x_diff*K_d
    z_response = head_error*K_p_z+head_integrator*K_i_z+head_diff*K_d_z
    if error_count>=len(error_norm)-1:
        error_count=0
    else:
        error_count+=1

    head_error
    error_norm[error_count]=np.linalg.norm((x_error,y_error,head_error*.125))
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
    # path = [(0,.1,0),(0,.2,30),(.2,.2,0),(.2,0,30),(0,0,0)]
    # path = [(0,0,0),(0,.1,0),(.1,.1,0),(.1,0,0),(0,0,0)]
    path = [(0,0,0),(0,.5,80),(.5,.5,0),(.5,0,30),(0,0,0)]
    path_num=0

    run_bool = True
    i=0
    

    while(path_num<len(path)):
        if odom_pid(path[path_num][0],path[path_num][1],path[path_num][2]):
            path_num+=1
        est_heading=yaw+initial_head
        est_x=robo_x+initial_x
        est_y=robo_y+initial_y
        est_heading_rad = np.deg2rad(est_heading)
        alpha_rad = np.arctan2(y_response,x_response)
        response_mag = np.linalg.norm((x_response,y_response))
        robo_x_speed = response_mag*np.cos(alpha_rad-est_heading_rad)
        robo_y_speed = response_mag*np.sin(alpha_rad-est_heading_rad)
        print("des: ({}, {}, {}) est:({:.2f}, {:.2f}, {:.2f})  response:({:.2f}, {:.2f}, {:.2f}) error: {:.2f} {:.2f} {:.2f}".format(path[path_num][0],path[path_num][1],path[path_num][2],est_x, est_y,est_heading,robo_x_speed,robo_y_speed,z_response,np.mean(error_norm),alpha_rad,response_mag))
        ep_chassis.drive_speed(-1*robo_x_speed,-1*robo_y_speed,-1*z_response,timeout=.1)
        time.sleep(.01)




    ep_chassis.unsub_attitude()
    ep_chassis.unsub_position()
    ep_sensor.unsub_distance()
    

    ep_robot.close()
        
