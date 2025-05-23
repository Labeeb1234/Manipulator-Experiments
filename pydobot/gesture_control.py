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
available_ports = list_ports.comports()
required_port = available_ports[-1].device
print(f"{required_port}")
dobot_mag = pydobot.Dobot(port=required_port, verbose=False)
#--------------------------------------------------------------------------------------


def constraint(val, clip_val):
    if val >= clip_val:
        return clip_val
    elif val <= -clip_val:
        return -clip_val
    else:
        return val

mutex_lock = threading.Lock()
max_data_points = 5
x_values=[]
y_values=[]
depth_values=[]
displacements = []



def realsense_handlandmark_tracking():

    hands = mp.solutions.hands.Hands(
        static_image_mode=False,
        max_num_hands = 1,
        min_detection_confidence=0.8,
        min_tracking_confidence=0.7
    )

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    align = rs.align(rs.stream.color)
    pipeline.start(config)

    prev_cx, prev_cy, dx, dy, prev_depth = 0, 0, 0, 0, 0
    alpha = 0.7 # emea noise smoother params
    alpha_d = 0.8 # noise smoother param for depth values

    try:
        while True:
            retry_attempts = 3
            frames = None

            for attempt in range(retry_attempts):
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=5000)  # 5 seconds timeout
                    break  # Break if frames are successfully retrieved
                except RuntimeError as e:
                    print(f"Attempt {attempt + 1}/{retry_attempts} failed: {e}")
                    if attempt < retry_attempts - 1:
                        time.sleep(1)  # Wait before retrying
                    else:
                        print("Failed to get frames after multiple attempts. Exiting.")
                        return
                    
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not color_frame or not depth_frame:
                continue

            # Convert images to numpy arrays so that cv2 can process it anyways mediapipe needs the frames in arrays
            aligned_frames = align.process(frames)
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
             
            start_time = time.perf_counter()
            results = hands.process(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
            end_time = time.perf_counter()
            fps = 1/(end_time-start_time)
            # color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
            cv2.putText(color_image, f'FPS:{int(fps)}', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)

            if results.multi_hand_landmarks:
                #pc.map_to(color_frame)
                #points = pc.calculate(depth_frame)
                #vertices = np.asanyarray(points.get_vertices())
                for result in results.multi_hand_landmarks:
                    x_min = min([lm.x for lm in result.landmark])               
                    y_min = min([lm.y for lm in result.landmark])
                    x_max = max([lm.x for lm in result.landmark])
                    y_max = max([lm.y for lm in result.landmark])

                    xtopLeft = x_min*color_image.shape[1]
                    ytopLeft = y_min*color_image.shape[0]
                    xbottomRight = x_max*color_image.shape[1]
                    ybottomRight = y_max*color_image.shape[0]

                    cx = (xtopLeft + xbottomRight)/2
                    cy = (ytopLeft + ybottomRight)/2

                    # ------------------ calibration points -----------------
                    # cv2.circle(color_image, (70,70), 1, (0, 255, 0), 2)
                    # cv2.circle(color_image, (200, 200), 1, (0, 255, 0), 2)
                    # -------------------------------------------------------
                    cv2.rectangle(color_image, (int(xtopLeft), int(ytopLeft)), (int(xbottomRight), int(ybottomRight)), (225, 0, 0), 2)
                    cv2.circle(color_image, (int(cx),int(cy)), 1, (255, 0, 0), 2)

                    with mutex_lock:
                        if 0 <= cx < color_image.shape[1] and 0 <= cy < color_image.shape[0]:
                            # EMEA filtered detected pixel values
                            smoothed_cx = alpha * prev_cx + (1 - alpha) * cx
                            smoothed_cy = alpha * prev_cy + (1 - alpha) * cy
                            dx = smoothed_cx - prev_cx
                            dy = smoothed_cy - prev_cy

                            depth = depth_image[int(smoothed_cy), int(smoothed_cx)]
                            smoothed_depth = alpha_d * prev_depth + (1-alpha_d)*depth

                            # Apply thresholds to ignore small movements
                            if abs(dx) < 5:
                                dx = 0
                            if abs(dy) < 5:
                                dy = 0

                            displacements.append((dx, dy, smoothed_depth))
                            if len(displacements) > max_data_points:
                                displacements.pop(0)

                            prev_cx, prev_cy, prev_depth = smoothed_cx, smoothed_cy, smoothed_depth

                        else:
                            print("cx or cy out of bounds.")
            # Show images
            cv2.imshow('Realsense RGB', color_image)
            # Exit loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Stop streaming and destroy window
        pipeline.stop()
        cv2.destroyAllWindows()



def display_data():
    while True:
        with mutex_lock:
            if displacements:
                dx, dy, depth_values = displacements[-1] # get latest values
                print(f"dx, dy, depth: [{dx}, {dy}, {depth_values}]")
        
        time.sleep(0.1)


def arm_control():
    plt.ion()
    fig, ax = plt.subplots()
    depth_history = []
    max_points = 10

    while True:
        with mutex_lock:
            if displacements:
                dx, dy, depth_values = displacements[-1]
                dx = int(constraint(dx, clip_val=10.0))
                dy = int(constraint(dy, clip_val=10.0))
                depth_values = int(depth_values)
                print(f"dx: {dx}, dy: {dy}, depth: {depth_values}")
                # might have to scale down dx and dy accordingly
                x, y, z, r, j1, j2, j3, j4 = dobot_mag.pose()
                dobot_mag.move_to(200, y, z, r, wait=False)

                depth_history.append(depth_values)
                if len(depth_history) > max_points:
                    depth_history.pop(0)

                # Update plot
                # ax.clear()
                # ax.plot(depth_history, label="Depth (mm)", color="blue")
                # ax.set_title("Depth Over Time")
                # ax.set_xlabel("Time Steps")
                # ax.set_ylabel("Depth (mm)")
                # ax.grid()
                # ax.legend()
                # plt.pause(0.01)
      
        time.sleep(0.1)  # Check displacement every 100ms



def main():
    try:
        tracking_thread = threading.Thread(target=realsense_handlandmark_tracking)
        tracking_thread.start()

        display_data_thread = threading.Thread(target=display_data)
        display_data_thread.start()

        # displacement_thread = threading.Thread(target=arm_control)
        # displacement_thread.start()

        tracking_thread.join()
        # displacement_thread.join()
        display_data_thread.join()
  
    except KeyboardInterrupt as e:
        print(f"User Interrupted Program!")

if __name__ == '__main__':
    main()