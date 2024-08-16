#!/usr/bin/env python3
import pyrealsense2 as rs
import cv2
import time
import mediapipe as mp
import numpy as np


def test_cam(device_id):
    hands = mp.solutions.hands.Hands(
        max_num_hands = 2,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.5
    )
    vid_cap = cv2.VideoCapture(device_id)

    while vid_cap.isOpened():
        ret, frame = vid_cap.read()
        frame = cv2.flip(frame, 1)
        if not ret:
            break
        
        
        results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if results.multi_hand_landmarks:
            for result in results.multi_hand_landmarks:
                print(f"{result}")
        
        
        cv2.imshow('frame', frame)
        



        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
    vid_cap.release()
    cv2.destroyAllWindows() 


def test_realsense(z_pose):
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))


    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    

    try:
        while True:
            try:
                # Wait for a coherent pair of frames: depth and color
                # Increased timeout to 10000 milliseconds (10 seconds)
                frames = pipeline.wait_for_frames(timeout_ms=10000)  

                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                if not color_frame or not depth_frame:
                    print("Error: Could not retrieve frames")
                    continue

                # Convert images to numpy arrays so that cv2 can process it anyways mediapipe needs the frames in arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                depth_image_dim = depth_image.shape
                color_colormap_dim = color_image.shape

                # converting pixel coordinates to real-world coordinates using depth (for reference using centre of whatever image is being displayed by realsense)
                x, y = color_image.shape[1]//2, color_image.shape[0]//2

                depth_value = depth_frame.get_distance(x,y) # in [m]
                
                z_pose.append(depth_value)

                depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # Convert depth to 8-bit image (important steps)
                depth_colormap = cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_JET)  # for Black-and-white colormap change from _JET ---> _BONE
                

                # Show images
                cv2.imshow('Realsense RGB', color_image)
                cv2.imshow('Realsense Depth Images', depth_colormap)

                # Exit loop on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except RuntimeError as e:
                print(f"RuntimeError: {e}")
                continue

    finally:
        # Stop streaming and destroy window
        pipeline.stop()
        cv2.destroyAllWindows()


def get_intrinsics_and_extrinsics():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable depth and color streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline_profile = pipeline.start(config)

    # Get the depth and color streams
    depth_stream = pipeline_profile.get_stream(rs.stream.depth)
    color_stream = pipeline_profile.get_stream(rs.stream.color)

    # Get the intrinsics (camera calibration information) for the depth and color streams
    depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
    color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

    # Print depth camera intrinsics
    print("Depth Camera Intrinsics:")
    print(f"  Width: {depth_intrinsics.width}")
    print(f"  Height: {depth_intrinsics.height}")
    print(f"  Focal Length (fx, fy): ({depth_intrinsics.fx}, {depth_intrinsics.fy})")
    print(f"  Principal Point (ppx, ppy): ({depth_intrinsics.ppx}, {depth_intrinsics.ppy})")
    print(f"  Distortion Coefficients: {depth_intrinsics.coeffs}")
    print(f"  Distortion Model: {depth_intrinsics.model}\n")

    # Print color camera intrinsics
    print("Color Camera Intrinsics:")
    print(f"  Width: {color_intrinsics.width}")
    print(f"  Height: {color_intrinsics.height}")
    print(f"  Focal Length (fx, fy): ({color_intrinsics.fx}, {color_intrinsics.fy})")
    print(f"  Principal Point (ppx, ppy): ({color_intrinsics.ppx}, {color_intrinsics.ppy})")
    print(f"  Distortion Coefficients: {color_intrinsics.coeffs}")
    print(f"  Distortion Model: {color_intrinsics.model}\n")

    # Get the extrinsics (the spatial transformation between depth and color streams)
    depth_to_color_extrinsics = depth_stream.get_extrinsics_to(color_stream)
    color_to_depth_extrinsics = color_stream.get_extrinsics_to(depth_stream)

    # Print extrinsics between depth and color cameras
    print("Depth to Color Camera Extrinsics:")
    print(f"  Rotation Matrix:\n{np.array(depth_to_color_extrinsics.rotation).reshape(3, 3)}")
    print(f"  Translation Vector: {depth_to_color_extrinsics.translation}")

    print("\nColor to Depth Camera Extrinsics:")
    print(f"  Rotation Matrix:\n{np.array(color_to_depth_extrinsics.rotation).reshape(3, 3)}")
    print(f"  Translation Vector: {color_to_depth_extrinsics.translation}")

    # Stop the pipeline
    pipeline.stop()





def realsense_handlandmark_tracking():
    pass


def main():
    get_intrinsics_and_extrinsics()

if __name__ == '__main__':
    main()