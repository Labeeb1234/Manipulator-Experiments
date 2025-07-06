#! /usr/bin/python3

import mediapipe as mp
import pyrealsense2 as rs
import time
import numpy as np
import cv2
import threading
from serial.tools import list_ports
import pydobot
from collections import deque
import matplotlib.pyplot as plt

#----------------------------Connecting to Device--------------------------------------
# available_ports = list_ports.comports()
# required_port = available_ports[-1].device
# print(f"{required_port}")
# dobot_mag = pydobot.Dobot(port=required_port, verbose=False)
#--------------------------------------------------------------------------------------


mutex_lock = threading.Lock()
MAX_POINTS = 300
x_values=[]
y_values=[]
depth_values=[]
displacements = []
landmarks = [] # optional

# plotting params
# Shared tracking data
dx_list = deque(maxlen=MAX_POINTS)
dy_list = deque(maxlen=MAX_POINTS)
darea_list = deque(maxlen=MAX_POINTS)
time_list = deque(maxlen=MAX_POINTS)

def constraint(val, clip_val):
    if val >= clip_val:
        return clip_val
    elif val <= -clip_val:
        return -clip_val
    else:
        return val
    
def rectangle_area(p1: tuple|np.ndarray, p2: tuple|np.ndarray)->float:
    return np.abs((p1[0]-p2[0])*(p1[1]-p2[1])) # square pixel units


def camera_handlandmark_tracking(vis=True):
    hands = mp.solutions.hands.Hands(
        static_image_mode=False,
        max_num_hands = 1,
        min_detection_confidence=0.8,
        min_tracking_confidence=0.7
    )

    vid_cap = cv2.VideoCapture(0)
    try:
        prev_x, prev_y, prev_area = 0, 0, 0
        t0 = time.time()

        while vid_cap.isOpened():
            ret, frame = vid_cap.read()
            frame = cv2.flip(frame, 1)
            if not ret:
                break
            
            start_time = time.perf_counter()
            results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            end_time = time.perf_counter()
            fps = 1/(end_time-start_time)
            cv2.putText(frame, f'FPS:{int(fps)}', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)
            
            if results.multi_hand_landmarks:
                H, W, = frame.shape[:2]
                for hand_landmarks in results.multi_hand_landmarks:
                    # bounding box around detected hand
                    xs, ys = zip(*[(lm.x, lm.y) for lm in hand_landmarks.landmark])
                    x_topleft, y_topleft = int(min(xs) * W), int(min(ys) * H)
                    x_bottomright, y_bottomright = int(max(xs) * W), int(max(ys) * H)
                    # getting the centre coordinates
                    # using mid-point theorem
                    area = rectangle_area(p1=(x_topleft,y_topleft), p2=(x_bottomright, y_bottomright))
                    cx, cy = (x_topleft+x_bottomright)//2, (y_topleft+y_bottomright)//2

                    # computing changes in quatities for tracking
                    dx, dy, darea = cx-prev_x, cy-prev_y, area-prev_area

                    # using all the landmarks to find the geometric centre of the hand faster 
                    # center_x = int(sum(lm.x for lm in hand_landmarks.landmark) / len(hand_landmarks.landmark) * W)
                    # center_y = int(sum(lm.y for lm in hand_landmarks.landmark) / len(hand_landmarks.landmark) * H)

                    if vis:
                        cv2.rectangle(frame, (x_topleft, y_topleft), (x_bottomright, y_bottomright), (255, 0, 0), 1)
                        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                    with mutex_lock:
                        dx_list.append(dx)
                        dy_list.append(dy)
                        darea_list.append(darea)
                        time_list.append(time.time() - t0)

                    prev_x, prev_y, prev_area = cx, cy, area


                    # # landmark annotations and tracking
                    # for lm in hand_landmarks.landmark:
                    #     x, y, _ = int(lm.x*W), int(lm.y*H), lm.z
                    #     # handlandmark annotation and tracking
                    #     cv2.circle(frame, (x,y), 4, (0, 255, 0), -1)

            if vis:      
                cv2.imshow("bgr camera frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        cv2.destroyAllWindows()


# def display_data():
#     while True:
#         with mutex_lock:
#             if displacements:
#                 dx, dy, depth_values = displacements[-1] # get latest values
#                 print(f"dx, dy, depth: [{dx}, {dy}, {depth_values}]")
        
#         time.sleep(0.1)

# Thread for live plotting
def display_data():
    plt.ion()
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6))
    while True:
        with mutex_lock:
            ax1.clear()
            ax2.clear()
            ax3.clear()

            ax1.plot(time_list, dx_list, label='dx')
            ax2.plot(time_list, dy_list, label='dy')
            ax3.plot(time_list, darea_list, label='Δarea')

        ax1.set_ylabel('dx')
        ax2.set_ylabel('dy')
        ax3.set_ylabel('Δarea')
        ax3.set_xlabel('Time (s)')

        ax1.legend()
        ax2.legend()
        ax3.legend()

        plt.pause(0.05)





def main():
    try:
        tracking_thread = threading.Thread(target=display_data)
        tracking_thread.start()

        camera_handlandmark_tracking(vis=True)

        # display_data_thread = threading.Thread(target=display_data)
        # display_data_thread.start()

        # tracking_thread.join()
        # display_data_thread.join()
  
    except KeyboardInterrupt as e:
        print(f"User Interrupted Program!")

if __name__ == '__main__':
    main()