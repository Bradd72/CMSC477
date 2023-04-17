import numpy as np
from robomaster import robot
import time




K_p = .0015;K_i = .00005;K_d=0;K_p_z = 45;K_i_z = .5;K_d_z = 5
x_error=0;x_integrator=0;x_response=0;est_x=0
y_error=0;y_integrator=0;y_response=0;est_y=0
z_error=0;z_integrator=0;z_response=0;est_heading=0
error_norm=np.ones((50,1))*100;error_tol=20;error_count=0
prev_time=time.time()
yaw=0; pitch=0; roll=0


def sub_attitude_info_handler(attitude_info):
    global yaw, pitch, roll
    yaw, pitch, roll = attitude_info

def sub_position_handler(position_info):
    est_x, y, z
    x, y, z = position_info

def odom_pid(des_X,des_Y,des_head):
    global y_diff,x_diff,y_integrator,x_integrator,y_error,x_error,y_response,x_response,error_count,error_norm,prev_time
    y_prev=y_error
    x_prev=x_error
    time_ = time.time()
    time_step = time_ - prev_time
    prev_time = time_
    if time_step < .5:
        y_integrator +=y_error*time_step   
        y_diff = (y_error - y_prev) / time_step
        x_integrator +=x_error*time_step  
        x_diff = (x_error - x_prev) / time_step
    else:
        y_integrator = 0
        y_diff = 0
        x_integrator = 0
        x_diff = 0
    head_error = (est_heading - des_head)
    y_error = (est_x - des_X)
    y_response = y_error*K_p+y_integrator*K_i+y_diff*K_d
    x_error = (est_y - des_Y)*-1
    x_response = x_error*K_p+x_integrator*K_i+x_diff*K_d
    if error_count>=len(error_norm)-1:
        error_count=0
    else:
        error_count+=1
    error_norm[error_count]=np.linalg.norm((x_error,y_error,head_error*500))
    if np.mean(error_norm)<error_tol: #
        error_norm=np.ones((50,1))*100
        return True
    else:
        return False

prev_time_est=0
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)


    run_bool = True
    des_x=10
    des_y=20
    des_head = 0
    while(run_bool):
        if odom_pid(des_x,des_y,des_head):
            ep_chassis.drive_speed(0,0,0)
            time.sleep(1)

        time_step_est = time.time() - prev_time_est
        prev_time_est = time.time()
        est_x+=time_step_est*(np.sin(est_heading)+)


    ep_chassis.unsub_attitude()

    ep_robot.close()
        
