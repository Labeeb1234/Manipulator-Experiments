#!/usr/bin/env python3
import os
import sys
import time
import threading
import numpy as np
import cv2 



sys.path.append(os.path.join(os.path.dirname(__file__),'../../..'))
from uarm.wrapper import SwiftAPI
from uarm.utils.log import logger

swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'}, callback_thread_pool_size=0, do_not_open=True)

def set_initial_endPose(swift, x, y, z):
    result = swift.set_position(x=x, y=y, z=z, wait=True, timeout=10)
    if result == 'OK':
        current_arm_pose = swift.get_position()
        print(f"x: {current_arm_pose[0]}, y: {current_arm_pose[1]}, z: {current_arm_pose[2]}")
        print("All commands completed successfully and robot is stopped.")
        return True
    elif result == 'TIMEOUT':
        print("Timed out waiting for commands to complete or robot to stop.")
        return False
    
    return False

def set_joint_angles(swift, angles, joint_id):
    for angle in angles:
        result = swift.set_servo_angle(joint_id, angle, wait=True, timeout=10)
    if result == 'OK':
        joint_angle = swift.get_servo_angle(joint_id)
        print(f"joint:{joint_id} angle:{joint_angle}")
        print("All commands completed successfully and robot is stopped.")
        return True
    elif result == 'TIMEOUT':
        print("Timed out waiting for commands to complete or robot to stop.")
        return False     
    return False
    
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

    
def end_effector_circle_motion(swift):
    curr_pose = swift.get_position()



def main():

    while True:

        try:
            # connecting to arm
            swift.connect(port='/dev/ttyACM0', baudrate=115200)
            # Wait until the robot is ready
            while(swift.waiting_ready() == False):
                print(f"waiting for arm to be ready.....")
            
            swift.set_speed_factor(1)
            swift.set_mode(0)
            # system status
            device_info = swift.get_device_info()
            device_power_info = swift.get_power_status()
            servo_info = swift.get_servo_attach()
            mode = swift.get_mode()
            gripper_status = swift.get_gripper_catch()
            
            if(mode == 0):
                pose_flag = swift.check_pos_is_limit([100.0, 50.0, 90.0])
                print(f"{pose_flag}")
                #func = set_initial_endPose(swift=swift, x=100.0, y=50.0, z=90.0)
                # if func:
                #     break
            

        except Exception as e:
            # Handle errors (e.g., connection issues)
            print(f"An error occurred: {str(e)}")
        
        finally:
            # Always disconnect after each loop iteration to avoid leaving the connection open
            swift.reset()
            time.sleep(5.0)
            swift.flush_cmd(timeout=10, wait_stop=True)
            swift.disconnect()
        
        # Wait until the next loop iteration
        time.sleep(0.1)




if __name__ == '__main__':
    main()