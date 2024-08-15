#!/usr/bin/env python3
import os
import sys
import time
import threading
import numpy as np
import cv2 

print(f"{cv2.__version__}")

sys.path.append(os.path.join(os.path.dirname(__file__),'../../..'))
from uarm.wrapper import SwiftAPI
from uarm.utils.log import logger

swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'}, callback_thread_pool_size=0, do_not_open=True)

def set_initial_endPose(swift, x, y, z):
    swift.set_position(x=x, y=y, z=z)
    result = swift.flush_cmd(timeout=10, wait_stop=True)

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
        joint_angle = swift.get_servo_angle()
        print("All commands completed successfully and robot is stopped.")
        return True
    elif result == 'TIMEOUT':
        print("Timed out waiting for commands to complete or robot to stop.")
        return False     
    return False
    
def main():

    while True:

        try:
            # connecting to arm
            swift.connect(port='/dev/ttyACM1', baudrate=115200)
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
                i = 0
                current_eepose = swift.get_position()
                func = set_joint_angles(swift=swift, angles=[10.0], joint_id=0)
                func2 = set_joint_angles(swift=swift, angles=[60.0], joint_id=1)
                if func and func2:
                    break
            

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