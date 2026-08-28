import pyrealsense2 as rs
import numpy as np
import cv2
from lib import PID


class CubeDetector:
    def __init__(self, kp=0.02, ki=0.0, kd=0.005):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.pid_x = PID(kp, ki, kd)
        self.pid_y = PID(kp, ki, kd)

    def get_frame(self):
        """Input: nothing | Output: frame"""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    def _get_color_masks(self, frame):
        """Input: frame | Output: combined_mask, individual_masks"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        yellow_mask = cv2.inRange(hsv, (18, 100, 100), (40, 255, 255))
        green_mask  = cv2.inRange(hsv, (35, 80, 50), (85, 255, 255))
        blue_mask   = cv2.inRange(hsv, (90, 100, 100), (130, 255, 255))
        red_mask1   = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        red_mask2   = cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))
        red_mask    = red_mask1 | red_mask2

        combined_mask = yellow_mask | green_mask | blue_mask | red_mask

        kernel = np.ones((5,5), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

        individual_masks = {
            "Yellow": yellow_mask,
            "Green": green_mask,
            "Blue": blue_mask,
            "Red": red_mask
        }
        return combined_mask, individual_masks

    def _get_contours(self, combined_mask):
        """Input: combined_mask | Output: contours"""
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def get_bbox(self, frame):
        """Input: frame, contours | Output: display frame with bboxes drawn"""
        display = frame.copy()
        combined_mask, _ = self._get_color_masks(frame)
        contours = self._get_contours(combined_mask)
        box = None
        for cnt in contours:
            if cv2.contourArea(cnt) < 3000:
                continue
            rect = cv2.minAreaRect(cnt)
            box  = cv2.boxPoints(rect)
            # box  = np.int32(box)
            cv2.drawContours(display, [cnt], 0, (0,255,0), 2)
        return box, display

    def get_color_detected(self, contours, combined_mask, individual_masks):
        """Input: contours, combined_mask, individual_masks | Output: color_map {(center_x, center_y): color}"""
        color_map = {}
        for cnt in contours:
            if cv2.contourArea(cnt) < 3000:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue

            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])

            contour_mask = np.zeros_like(combined_mask)
            cv2.drawContours(contour_mask, cnt, -1, 255, -1)

            color_detected = "Unknown"
            max_pixels = 0
            for color_name, mask in individual_masks.items():
                masked = cv2.bitwise_and(mask, mask, mask=contour_mask)
                pixels = cv2.countNonZero(masked)
                if pixels > max_pixels and pixels > 1000:
                    max_pixels = pixels
                    color_detected = color_name

            color_map[(center_x, center_y)] = color_detected

        return color_map

    def get_best_center(self, frame):
        """Input: frame, contours | Output: best_center_x, best_center_y"""
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        best_error    = None
        best_center_x = None
        best_center_y = None

        combined_mask, _ = self._get_color_masks(frame)
        contours = self._get_contours(combined_mask)
        for cnt in contours:
            if cv2.contourArea(cnt) < 3000:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue

            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            error    = frame_center_x - center_x

            if best_error is None or abs(error) < abs(best_error):
                best_error    = error
                best_center_x = center_x
                best_center_y = center_y

        return best_center_x, best_center_y, frame_center_x, frame_center_y

    def get_best_error(self, frame, best_center_x, best_center_y):
        """Input: frame, best_center_x, best_center_y | Output: best_error (closest to zero)"""
        if best_center_x is None or best_center_y is None:
            return None
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2
        error_x = frame_center_x - best_center_x
        error_y = frame_center_y - best_center_y
         #best_error = error_x if abs(error_x) < abs(error_y) else error_y
        return error_x, error_y

    def get_pid_output(self, frame, best_center_x, best_center_y):
        """Input: frame, best_center_x, best_center_y | Output: pid_output_x, pid_output_y"""
        if best_center_x is None or best_center_y is None:
            return None, None
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        error_x = frame_center_x - best_center_x
        error_y = frame_center_y - best_center_y

        pid_output_x = self.pid_x.update(error_x)
        pid_output_y = self.pid_y.update(error_y)

        return pid_output_x, pid_output_y

    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()