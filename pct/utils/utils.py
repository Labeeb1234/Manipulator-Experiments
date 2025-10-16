import time
import functools
import cv2
import numpy as np
from typing import Any

# --------------------------------------------COMPUTER VISION UTILS ------------------------------------------------
def canny_edge_detection(img_bgr: np.ndarray):
    ''' Function basically does the canny edge detection to detect the figures and features in the input image
        and returns the raw binary img and cleaned binary image as a tuple (tuned algo but can and maybe be tuned further)'''
    
    # grayscale conversion
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    # using gaussian blur to remove noise from the image (tuned to handle images within 0.1sigma std-dev)
    smoothed = cv2.GaussianBlur(gray, (5,5), 0.1) # ksize is also tuned here
    # canny edge detection
    edges = cv2.Canny(smoothed, threshold1=100, threshold2=200) # tuned with focus of extracting the pentagon figures mainly
    # morphological close to enhance the detected features
    kernel = np.ones((5,5), np.uint8)
    cleaned_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    return edges, cleaned_edges # the binary output tuple(raw edges, cleaned edges) 

def binarize_img(img_bgr: np.ndarray):
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    # adaptive mean thresholding for binarization
    c_val = 17
    binary_img = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 93, c_val)

    # cleaning up the binary image morphologically
    # Opening = Erosion followed by Dilation. Removes small white noise (speckles).
    # Define the kernel (a 2x2 or 3x3 square kernel is common for noise)
    kernel = np.ones((1, 1), np.uint8) 
    # Apply the morphological opening operation
    cleaned_binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, kernel)

    return binary_img, cleaned_binary_img

def get_point_distance(p1: np.ndarray, p2: np.ndarray):
    return np.linalg.norm(p1-p2) # pixel units (taking manhattan distance)

def approximate_polygon(contour: Any, config: dict):
    # ------------------- TUNABLE PARAMS ----------------------
    EPSILON_FACTOR = config["EPSILON_FACTOR"] # 0.02 # perimeter factor
    # ------------------------------------------------
    epsilon = EPSILON_FACTOR*cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    num_vertices = len(approx)
    vertices = []
    for vertex in approx:
        vx,vy = vertex[0]
        vertices.append((vx,vy))
    return approx, num_vertices, vertices # returning collection of approximated polygon points


def point_on_segment(x, y, xA, yA, xB, yB, eps):
    return (min(xA, xB) - eps <= x <= max(xA, xB) + eps and min(yA, yB) - eps <= y <= max(yA, yB) + eps) # check if the points inside the line segment

def get_intersection_point(line1, line2, config: dict, method=0):
    # ------------------- TUNABLE PARAMS ----------------------------------------
    PARALLEL_THRESHOLD = config["PARALLEL_THRESHOLD"] # parallel_threshold=1e-10, eps=1.0e-10, 
    EPSILON = config["EPSILON"] # small real positive value range==>[0-1]
    #--------------------------------------------------------------------
    # 0-houghlineP and 1-houghlines
    if method == 0:
        rho1, theta1 = line1[0]
        rho2, theta2 = line2[0]

        A = np.array([
            [np.cos(theta1), np.sin(theta1)],
            [np.cos(theta2), np.sin(theta2)]
        ])
        b = np.array([rho1, rho2])

        if np.linalg.det(A) > PARALLEL_THRESHOLD:
            inter_x, inter_y = np.linalg.solve(A, b) 
            return float(inter_x), float(inter_y)
        else:
            return None
    else:
        x1,y1,x2,y2 = line1[0]
        x3,y3,x4,y4 = line2[0]
        # cramers rule to check for parallelity
        a1 = (y2-y1)
        b1 = (x1-x2)
        c1 = (a1*x1 + b1*y1)

        a2 = (y4-y3)
        b2 = (x4-x3)
        c2 =  a2*x3 + b2*y3

        # use this or use the numpy vectorization to solve and check for parallelity (like shown above)
        det = a1*b2-a2*b1 # det(A) == > AX = C
        # Use parallel_threshold instead of exact zero
        if np.abs(det) > PARALLEL_THRESHOLD:
            Dx = c1*b2 - c2*b1
            Dy = a1*c2 - a2*c1
            inter_x = Dx / det
            inter_y = Dy / det
            if (point_on_segment(inter_x, inter_y, x1, y1, x2, y2, eps=EPSILON) and point_on_segment(inter_x, inter_y, x3, y3, x4, y4, eps=EPSILON)):
                return float(inter_x), float(inter_y)
        else:
            return None

# ------------------------------------------------------------------------------------------------------------------

# ------------------------------- ALGO UTILS -----------------------------------
def get_contour_info(img_bgr: np.ndarray, config: dict, contouring_mode=cv2.RETR_EXTERNAL):
    ann_img = img_bgr.copy()
    # bin_img, cleaned_bin_img = binarize_img(img_bgr=img_bgr)
    edges, cleaned_edges = canny_edge_detection(img_bgr=img_bgr)
    contours, heirarchy = cv2.findContours(cleaned_edges, contouring_mode, cv2.CHAIN_APPROX_NONE)
    
    # --------------- TUNABLE PARAMS -----------------
    MIN_CONTOUR_AREA = config["MIN_CONTOUR_AREA"] # 5000 # pix^2 
    MIN_SOLIDITY = config["MIN_SOLIDITY"] # 0.75
    #-----------------------------------------

    roi_info = {f"C{i}": [] for i in range(len(contours))} # extra information
    for idx, cnt in enumerate(contours):
        cnt_area = cv2.contourArea(cnt)
        if cnt_area < MIN_CONTOUR_AREA:
            continue
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = float(cnt_area)/hull_area
        if solidity < MIN_SOLIDITY:
            continue
        
        x,y,w,h = cv2.boundingRect(cnt)
        roi_info[f"C{idx}"].append([x,y,w,h])
        # roi_info[f"C{idx}"] = {
        #     "bbox": [x,y,w,h],
        #     "ext_contour": cnt
        # } # important stuff
        cv2.drawContours(ann_img, cnt, -1, (0,255,0), 1)
        cv2.rectangle(ann_img, (x,y), (x+w,y+h), (255,0,0), 1)
        cv2.putText(ann_img, f"C{idx}", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)
    return ann_img, roi_info

# ---------------------------------------------------------------------------------


# ----------------------- EXTRA STUFF -----------------------
# for displaying already set params for main app only
def display_ascii_params(config: dict):
    """Pretty print params as an ASCII table."""
    print("=" * 45)
    print(f"{'CURRENT QIP PARAMETER STATUS':^45}")
    print("=" * 45)
    print(f"{'Parameter':<25} | {'Value':>15}")
    print("-" * 45)
    for k, v in config.items():
        if isinstance(v, float):
            print(f"{k:<25} | {v:>15.4f}")
        else:
            print(f"{k:<25} | {v:>15}")
    print("=" * 45)
    print(f"\n")

# for function performance logging for main app only
def log_perf(func):
    """
    Decorator to log entry, exit, and execution time of a function.
    """
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        print(f"[ENTER] Function '{func.__name__}' called.")
        start_time = time.perf_counter()  # High-resolution timer
        
        result = func(*args, **kwargs)
        
        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        print(f"[EXIT] Function '{func.__name__}' finished. [Elapsed time: {elapsed_time*1000:.3f} ms]\n")
        
        return result
    return wrapper
# ------------------------------------------------------------

