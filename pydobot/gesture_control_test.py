#! /usr/bin/python3

import mediapipe as mp
import pyrealsense2 as rs
import time
import numpy as np
import cv2
import threading
from serial.tools import list_ports
import pydobot
import matplotlib.pyplot as plt

#----------------------------Connecting to Device--------------------------------------
# available_ports = list_ports.comports()
# required_port = available_ports[-1].device
# print(f"{required_port}")
# dobot_mag = pydobot.Dobot(port=required_port, verbose=False)
#--------------------------------------------------------------------------------------


def constraint(val, clip_val):
    if val >= clip_val:
        return clip_val
    elif val <= -clip_val:
        return -clip_val
    else:
        return val
    
def rectangle_area(p1: tuple|np.ndarray, p2: tuple|np.ndarray)->float:
    return np.abs((p1[0]-p2[0])*(p1[1]-p2[1])) # square pixel units

mutex_lock = threading.Lock()
max_data_points = 5
x_values=[]
y_values=[]
depth_values=[]
displacements = []
landmarks = [] # optional


def camera_handlandmark_tracking():
    hands = mp.solutions.hands.Hands(
        static_image_mode=False,
        max_num_hands = 1,
        min_detection_confidence=0.8,
        min_tracking_confidence=0.7
    )

    vid_cap = cv2.VideoCapture(0)
    try:
        prev_x, prev_y, prev_area = 0, 0, 0

        while vid_cap.isOpened():
            ret, frame = vid_cap.read()
            if not ret:
                break
            
            start_time = time.perf_counter()
            results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            end_time = time.perf_counter()
            fps = 1/(end_time-start_time)
            cv2.putText(frame, f'FPS:{int(fps)}', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)
            
            if results.multi_hand_landmarks:
                height, width, _ = frame.shape
                for hand_landmarks in results.multi_hand_landmarks:
                    # bounding box around detected hand
                    x_list = [lm.x for lm in hand_landmarks.landmark]
                    y_list = [lm.y for lm in hand_landmarks.landmark]
                    x_topleft = int(min(x_list) * width)
                    y_topleft = int(min(y_list) * height)
                    x_bottomright = int(max(x_list) * width)
                    y_bottomright = int(max(y_list) * height)
                    cv2.rectangle(frame, (x_topleft, y_topleft), (x_bottomright, y_bottomright), (255, 0, 0), 1)
                    
                    with mutex_lock: 
                        area = rectangle_area((x_topleft, y_topleft), (x_bottomright, y_bottomright))
                        darea = area-prev_area
                        cx, cy = (x_topleft+x_bottomright)//2, (y_topleft+y_bottomright)//2
                        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                        dx, dy = cx-prev_x, cy-prev_y
                        print(f"dx: {dx}, dy: {dy}")

                    # landmark annotations and tracking
                    for lm in hand_landmarks.landmark:
                        x, y, _ = int(lm.x*width), int(lm.y*height), lm.z
                        # handlandmark annotation and tracking
                        cv2.circle(frame, (x,y), 4, (0, 255, 0), -1)
                    
            cv2.imshow("bgr camera frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()


def display_data():
    while True:
        with mutex_lock:
            if displacements:
                dx, dy, depth_values = displacements[-1] # get latest values
                print(f"dx, dy, depth: [{dx}, {dy}, {depth_values}]")
        
        time.sleep(0.1)


# def arm_control():
#     plt.ion()
#     fig, ax = plt.subplots()
#     depth_history = []
#     max_points = 10

#     while True:
#         with mutex_lock:
#             if displacements:
#                 dx, dy, depth_values = displacements[-1]
#                 dx = int(constraint(dx, clip_val=10.0))
#                 dy = int(constraint(dy, clip_val=10.0))
#                 depth_values = int(depth_values)
#                 print(f"dx: {dx}, dy: {dy}, depth: {depth_values}")
#                 # might have to scale down dx and dy accordingly
#                 x, y, z, r, j1, j2, j3, j4 = dobot_mag.pose()
#                 dobot_mag.move_to(200, y, z, r, wait=False)

#                 depth_history.append(depth_values)
#                 if len(depth_history) > max_points:
#                     depth_history.pop(0)

#                 # Update plot
#                 # ax.clear()
#                 # ax.plot(depth_history, label="Depth (mm)", color="blue")
#                 # ax.set_title("Depth Over Time")
#                 # ax.set_xlabel("Time Steps")
#                 # ax.set_ylabel("Depth (mm)")
#                 # ax.grid()
#                 # ax.legend()
#                 # plt.pause(0.01)
      
#         time.sleep(0.1)  # Check displacement every 100ms



def main():
    try:
        tracking_thread = threading.Thread(target=camera_handlandmark_tracking)
        tracking_thread.start()

        display_data_thread = threading.Thread(target=display_data)
        display_data_thread.start()

        tracking_thread.join()
        display_data_thread.join()
  
    except KeyboardInterrupt as e:
        print(f"User Interrupted Program!")

if __name__ == '__main__':
    main()