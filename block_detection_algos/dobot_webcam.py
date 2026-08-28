import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

def get_color_masks(hsv):

    yellow_mask = cv2.inRange(hsv, (18, 100, 100), (40, 255, 255))
    green_mask  = cv2.inRange(hsv, (40, 80, 80), (85, 255, 255))
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

while True:
    ret, frame = cap.read()
    if not ret:
        break

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

    for cnt in contours:

        area = cv2.contourArea(cnt)

        # Ignore small noise
        if area < 3000:
            continue

    
        M = cv2.moments(cnt)     #stable center using moments
        if M["m00"] == 0:
            continue

        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])

    
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

        
        frame_center = frame.shape[1] // 2     #pid 
        error = frame_center - center_x



        cv2.putText(display,
                    f"Error: {error}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0,0,255),
                    2)

        print("Color:", color_detected, "CenterX:", center_x, "Error:", error)

    cv2.imshow("Stable Multi-Color Cube Detection", display)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()     