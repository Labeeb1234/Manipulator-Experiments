import pyrealsense2 as rs
import numpy as np
import cv2
import threading
from lib import PID





pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
pid_x = PID(0.02, 0.0, 0.005)
pid_y = PID(0.02, 0.0, 0.005)

def get_color_masks(hsv):
    yellow_mask = cv2.inRange(hsv, (18, 100, 100), (40, 255, 255))
    green_mask  = cv2.inRange(hsv, (35, 80, 50), (85, 255, 255))
    blue_mask   = cv2.inRange(hsv, (90, 100, 100), (130, 255, 255))

    red_mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
    red_mask2 = cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))
    red_mask = red_mask1 | red_mask2

    combined_mask = yellow_mask | green_mask | blue_mask | red_mask
    return combined_mask, {
        "Yellow": yellow_mask,
        "Green": green_mask,
        "Blue": blue_mask,
        "Red": red_mask
    }

def object_detection_inference():
    while True: 
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            return

        frame = np.asanyarray(color_frame.get_data())
        display = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        combined_mask, individual_masks = get_color_masks(hsv)

        kernel = np.ones((5,5), np.uint8)                                         # clean mask
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            combined_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        best_error=None
        best_center_x=None
        best_center_y=None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Ignore small noise
            if area < 3000:
                continue

            # Configure pipeline
            M = cv2.moments(cnt)     #stable center using moments
            if M["m00"] == 0:
                continue

            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            pixel = hsv[center_y, center_x]
            # print("HSV:", pixel)

            rect = cv2.minAreaRect(cnt)     #bounding box
            box = cv2.boxPoints(rect)
            box = np.int32(box)

            cv2.drawContours(display, [box], 0, (0,255,0), 2)
            cv2.circle(display, (center_x, center_y), 5, (255,0,0), -1)

            contour_mask = np.zeros_like(combined_mask)      #empty mask            
            cv2.drawContours(contour_mask, [cnt], -1, 255, -1)   # contour drawing

            color_detected = "Unknown"
            max_pixels = 0
            for color_name, mask in individual_masks.items():
                
                masked = cv2.bitwise_and(mask, mask, mask=contour_mask)    # for contour mask
                pixels = cv2.countNonZero(masked)

                if pixels > max_pixels:
                    max_pixels = pixels
                    color_detected = color_name
                if max_pixels < 1000:   # tune this
                    continue     
                cv2.putText(display,
                            color_detected,
                            (center_x - 40, center_y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0,255,0),
                            2)

            
            frame_center_x = frame.shape[1] // 2     #pid 
            frame_center_y = frame.shape[0] // 2
            error = frame_center_x - center_x

            if best_error is None or abs(error) < abs(best_error):
                best_error = error
                best_center_x = center_x
                best_center_y = center_y
            if best_error is not None:
                error_x = frame_center_x - best_center_x
                error_y = frame_center_y - best_center_y
                pid_output_x = pid_x.update(error_x)
                pid_output_y = pid_y.update(error_y)
        cv2.putText(display,
                    f"Error: {best_error}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0,0,255),
                    2)
        
        cv2.circle(display, (frame_center_x, frame_center_y), 5, (0,0,255), -1)
        # print("Color:", color_detected, "CenterX:", center_x, "Error:", best_error, "PID_output_x:", pid_output_x, "PID_output_y:", pid_output_y)

        cv2.imshow("Stable Multi-Color Cube Detection", display)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    pipeline.stop()
    cv2.destroyAllWindows()

    def start(self):
        
        self._running = True
        self._thread = threading.Thread(target = object_detection_inference, daemon = True)
        self._thread.start()

    def get_center(self):
        with self._lock:
            return self.best_center_x, self.best_center_y

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()