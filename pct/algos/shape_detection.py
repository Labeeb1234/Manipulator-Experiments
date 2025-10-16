import numpy as np
import cv2

def plain_shape_detection(img_bgr: np.ndarray):
    annotated_img = img_bgr.copy()
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY) #  converting image to binary format that's it (normal thresholding) --> can change to OSTU thresholding or adaptive

    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # add contour morphological cleaning if necessary

    # ----------------------------------------------
    PERIMETER_FACTOR = 0.01
    # Process each contour
    for i, contour in enumerate(contours):
        cnt_area = cv2.contourArea(contour)
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            continue

        # checking solidity
        solidity = float(cnt_area)/hull_area
        if solidity < 0.05:
            continue

        perimeter = PERIMETER_FACTOR * cv2.arcLength(contour, True) # epsilon value
        approx = cv2.approxPolyDP(contour, perimeter, True)
        for vertex in approx:
            x, y = vertex[0]
            # print(f"Vertex: ({x}, {y})")
            cv2.circle(annotated_img, (x, y), 5, (0, 255, 0), -1)

        # Draw contour
        cv2.drawContours(annotated_img, [contour], 0, (0, 0, 255), 2)
        # tight bbox around contours
        x_bbox, y_bbox, w_bbox, h_bbox = cv2.boundingRect(contour)
        # Draw the bounding box (color: Red, thickness: 2)
        cv2.rectangle(annotated_img, (x_bbox, y_bbox), (x_bbox + w_bbox, y_bbox + h_bbox), (255, 0, 0), 2)

        # Find center (point of the contour)
        M = cv2.moments(contour)
        if M['m00'] != 0:
            x = int(M['m10'] / M['m00'])
            y = int(M['m01'] / M['m00'])

        # Detect shape
        label=f"cnt-{i}"
        # Label the shape
        cv2.putText(annotated_img, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


    return annotated_img


