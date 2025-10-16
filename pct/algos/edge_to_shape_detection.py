import cv2
import numpy as np
from utils import canny_edge_detection


def verify_edge_detection_integrity(img_bgr: np.ndarray):
    ann_img = img_bgr.copy()
    edges, cleaned_edges = canny_edge_detection(img_bgr=img_bgr)
    contours, heirarchy = cv2.findContours(cleaned_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    print(f"[INFO]: contours detected: {len(contours)}")

    # ---- set contour filtering params ---- 
    MIN_CONTOUR_AREA = 5000
    MIN_SOLIDITY = 0.85 # range=[0, 1] 1 means the contours are more convex
    # --------------------------------------

    for i, cnt in enumerate(contours):
        cnt_area = cv2.contourArea(cnt)
        if cnt_area < 5000:
            continue
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = float(cnt_area)/hull_area
        if solidity < 0.85:
            continue

        cv2.drawContours(ann_img, cnt, -1, (0, 0, 255), 1)
        

    return ann_img
