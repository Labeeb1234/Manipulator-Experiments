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

def low_pass_filter(curr_val, prev_val, alpha=0.6): # emea filter
    curr_val = alpha*curr_val + (1-alpha)*prev_val
    return curr_val

def dead_zone_filter(val, threshold):
    if abs(val) < threshold:
        return 0.0
    return val


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
        prev_dx_filt, prev_dy_filt, prev_darea_filt = 0, 0, 0
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
                    # xyXY
                    x_topleft, y_topleft = int(min(xs) * W), int(min(ys) * H)
                    x_bottomright, y_bottomright = int(max(xs) * W), int(max(ys) * H)
                    # getting the centre coordinates
                    # using mid-point theorem
                    area = rectangle_area(p1=(x_topleft,y_topleft), p2=(x_bottomright, y_bottomright))
                    # cx, cy = (x_topleft+x_bottomright)//2, (y_topleft+y_bottomright)//2
                    # using all the landmarks to find the geometric centre of the hand faster 
                    cx = int(sum(lm.x for lm in hand_landmarks.landmark) / len(hand_landmarks.landmark) * W)
                    cy = int(sum(lm.y for lm in hand_landmarks.landmark) / len(hand_landmarks.landmark) * H)

                    # computing changes in quatities for tracking
                    dx, dy, darea = cx-prev_x, cy-prev_y, area-prev_area
                    dx = constraint(dx, clip_val=1.0)
                    dy = constraint(dy, clip_val=1.0)
                    darea = constraint(darea, clip_val=5.0)

                    # adding a low-pass filter for the pixel_changes
                    dx = low_pass_filter(dx, prev_dx_filt)
                    dy = low_pass_filter(dy, prev_dy_filt)
                    darea = low_pass_filter(darea, prev_darea_filt, alpha=0.4)
                    prev_dx_filt, prev_dy_filt, prev_darea_filt = dx, dy, darea

                    # # Apply dead zone thresholding
                    # dx = dead_zone_filter(dx, threshold=0.6)
                    # dy = dead_zone_filter(dy, threshold=0.6)
                    # darea = dead_zone_filter(darea, threshold=3.3)                    
                    if vis:
                        cv2.rectangle(frame, (x_topleft, y_topleft), (x_bottomright, y_bottomright), (255, 0, 0), 1)
                        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                    with mutex_lock:
                        dx_list.append(dx)
                        dy_list.append(dy)
                        darea_list.append(darea)
                        time_list.append(time.time() - t0)

                    prev_x, prev_y, prev_area = cx, cy, area
                    
            if vis:      
                cv2.imshow("bgr camera frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        cv2.destroyAllWindows()


# Thread for live plotting
def visualize_data(axis_x=True, axis_y=True, axis_area=True):
    plt.ion()
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6))
    while True:
        with mutex_lock:
            ax1.clear()
            ax2.clear()
            ax3.clear()
            if axis_x:
                ax1.plot(time_list, dx_list, label='dx')
            if axis_y:
                ax2.plot(time_list, dy_list, label='dy')
            if axis_area:
                ax3.plot(time_list, darea_list, label='Δarea')

        ax1.set_ylabel('dx')
        ax2.set_ylabel('dy')
        ax3.set_ylabel('Δarea')
        ax3.set_xlabel('Time (s)')

        ax1.legend()
        ax2.legend()
        ax3.legend()

        plt.pause(0.01)

def analyze_noise():
    # using std for finding noise
    avg_dx_std, avg_dy_std, avg_darea_std = 0.0, 0.0, 0.0
    count = 0

    for _ in range(100):
        with mutex_lock:
            if len(dx_list) == 0:
                time.sleep(0.1)
                continue
            dx_std = np.std(dx_list)
            dy_std = np.std(dy_list)
            darea_std = np.std(darea_list)
            avg_darea_std += darea_std
            avg_dx_std += dx_std
            avg_dy_std += dy_std
            count += 1
        print(f"[Noise STD] dx: {dx_std:.4f}, dy: {dy_std:.4f}, Δarea: {darea_std:.4f}")
        time.sleep(0.1) # 10Hz

    if count > 0:
        print("\n=== Average Noise STD over {} samples ===".format(count))
        print(f"Avg dx std: {avg_dx_std / count:.4f}")
        print(f"Avg dy std: {avg_dy_std / count:.4f}")
        print(f"Avg Δarea std: {avg_darea_std / count:.4f}")
    else:
        print("No data collected to calculate average noise.")
    
    


def main():
    try:
        tracking_thread = threading.Thread(target=visualize_data)
        tracking_thread.start()
        camera_handlandmark_tracking(vis=True)

    except KeyboardInterrupt as e:
        print(f"User Interrupted Program!")

if __name__ == '__main__':
    main()