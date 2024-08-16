#!/usr/bin/env python3
import os
import sys
import time
import threading
import numpy as np
from math import sin, cos
import cv2 

sys.path.append(os.path.join(os.path.dirname(__file__),'../../..'))
from uarm.wrapper import SwiftAPI
from uarm.utils.log import logger

swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'}, callback_thread_pool_size=0, do_not_open=True)

def set_eepose(swift, x, y, z):
    swift.set_position(x=x, y=y, z=z) # speed_val in mm/min
    result = swift.flush_cmd(timeout=10, wait_stop=True) # ---> use this its better
    if result == 'OK':
        print(f"{swift.get_position()}")
        print("All commands completed successfully and robot is stopped.")
        return True
    elif result == 'TIMEOUT':
        print("Timed out waiting for commands to complete or robot to stop.")
        return False
    
    return False

def set_joint_angles(swift, angles, joint_id):
    for angle in angles:
        swift.set_servo_angle(joint_id, angle)
        result = swift.flush_cmd(timeout=10, wait_stop=True)
        if result == 'OK':
            joint_angle = swift.get_servo_angle(joint_id)
            print(f"joint-{joint_id}-->angle:{joint_angle}")
            print("All commands completed successfully and robot is stopped.")
            return True
        elif result == 'TIMEOUT':
            print("Timed out waiting for commands to complete or robot to stop.")
            return False     
    return False

def set_wrist_joint_angle(swift, angle):
    result = swift.set_wrist(angle, wait=True, timeout=10)
    if result == 'OK':
        joint_angle = swift.get_servo_angle(4)
        print(f"joint-{4}-->angle:{joint_angle}")
        print("All commands completed successfully and robot is stopped.")
        return True
    elif result == 'TIMEOUT':
        print("Timed out waiting for commands to complete or robot to stop.")
        return False     
    return False  

def set_home_position(swift):
    set_eepose(swift, x=196.96, y=7.25, z=16.27)

def end_effector_circle_motion(swift, start_flag, control_freq):
    set_home_position(swift=swift)
    # using counter val method
    previous_loop_time = time.time()
    try:
        while(start_flag):
            if ((time.time()-previous_loop_time) > (1/control_freq)):
                #print(f"{previous_loop_time}")
                curr_pose = swift.get_position()
                curr_x = curr_pose[0]
                curr_y = curr_pose[1]
                curr_z = curr_pose[2]
                x_pose = 10*cos(time.time()) + curr_x
                y_pose = 10*sin(time.time()) + curr_y
                set_eepose(swift, x=x_pose, y=y_pose, z=90.0)
                previous_loop_time = time.time()
    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Stopping motion and returning to home position.")
    finally:
        set_home_position(swift)
        print("Manipulator moved to home position.")
        return True     


    
def test_cam(device_id):
    vid_cap = cv2.VideoCapture(device_id)
    while(vid_cap.isOpened()):
        ret_val, frame = vid_cap.read()
        frame = cv2.flip(frame,1)

        if not ret_val:
            break

        height, width, channel = frame.shape

        # Dot properties
        dot_radius = 10
        dot_color = (0, 0, 255)  # Red color in BGR

        # Define corner positions
        corners = [
            (dot_radius, dot_radius),  # Top-left corner
            (width - dot_radius, dot_radius),  # Top-right corner
            (dot_radius, height - dot_radius),  # Bottom-left corner
            (width - dot_radius, height - dot_radius)  # Bottom-right corner
        ]
        # print(f"{corners}") ---> [(10, 10), (630, 10), (10, 470), (630, 470)]

        # Draw dots at each corner
        for corner in corners:
            cv2.circle(frame, corner, dot_radius, dot_color, -1)

        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
    vid_cap.release()
    cv2.destroyAllWindows()


def main():

    while True:
        try:
            # connecting to arm
            swift.connect(port='/dev/ttyACM1', baudrate=115200)
            # Wait until the robot is ready
            while(swift.waiting_ready() == False):
                print(f"waiting for arm to be ready.....")  
            # speed scale factor
            swift.set_speed_factor(1.0)
            swift.set_mode(0)
            # system feedback
            servo_info = swift.get_servo_attach()
            mode = swift.get_mode()
            # set_joint_angles(swift, [90.0], 0)
    
            # motion_flag = end_effector_circle_motion(swift, True, 100.0)
            # if motion_flag == True:
            #     break

        except Exception as e:
            # Handle errors (e.g., connection issues)
            print(f"An error occurred: {str(e)}")
        
        finally:
            # Always disconnect after each loop iteration to avoid leaving the connection open
            swift.disconnect()
        
        # Wait until the next loop iteration
        time.sleep(0.1)




if __name__ == '__main__':
    main()