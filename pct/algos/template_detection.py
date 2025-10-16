# differentiating template diagram from hand-drawn diagram

# just a test based on an observation of few samples

import numpy as np
import cv2
from utils import binarize_img

def extract_polygon_and_contours(img_bgr: np.ndarray):
    ann_img = img_bgr.copy()
    # outermost contour detection from edges detected
    edges, cleaned_edges = binarize_img(img_bgr=img_bgr)

    contours, _ = cv2.findContours(cleaned_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    print(f"Number of detected contours: {len(contours)}")

    # ---- set contour filtering params ---- 
    MIN_CONTOUR_AREA = 5000
    MIN_SOLIDITY = 0.75 # range=[0, 1] 1 means the contours are more convex
    PERIMETER_FACTOR = 0.02 # epsilong factor
    # --------------------------------------

    candidate_contours = []
    estimated_ply_contours = [] # will be a linearized structure/contours (i.e no curves just an estimated polygons reconstructed via lines drawn between vertices approximated)
    for i, cnt in enumerate(contours):
        cnt_area = cv2.contourArea(cnt)
        if cnt_area < MIN_CONTOUR_AREA:
            continue
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = float(cnt_area)/hull_area
        if solidity < MIN_SOLIDITY:
            continue
    
        candidate_contours.append(cnt)
        # combined outer polygon approximation
        epsilon = PERIMETER_FACTOR*cv2.arcLength(cnt, True) # weighted perimeter
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        estimated_ply_contours.append(approx)
        # sanity check on the figure approximated polygon points ---> (should be 12(printed)+12(hand-drawn) ideally)
        for j, vertices in enumerate(approx):
            vx, vy = vertices[0]
            center = (int(vx), int(vy))
            # Draw the vertex point
            label = f"v{j}"
            cv2.circle(ann_img, center, 5, (255, 0, 0), -1)
            cv2.putText(ann_img, f"{label}", (center), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)
        
    return ann_img, candidate_contours, estimated_ply_contours

def differentiate_template_figure(img_bgr:np.ndarray):
    ann_img, contours, poly_approx = extract_polygon_and_contours(img_bgr=img_bgr)
    for i, (cnt, poly) in enumerate(zip(contours, poly_approx)):
        match_shape_value = cv2.matchShapes(cnt, poly, cv2.CONTOURS_MATCH_I3, 0)
        print(f"Contour {i+1} Match Score (Original vs. Polygon): {match_shape_value:.6f}")
        # annotating both contours and the polygon approx
        cv2.drawContours(ann_img, cnt, -1, (0,0,255), 1)
        cv2.drawContours(ann_img, [poly], -1, (0, 255, 0), 1)
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.putText(ann_img, f"Match: {match_shape_value:.4f}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return ann_img