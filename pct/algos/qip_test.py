import numpy as np
import cv2

from utils import log_perf, canny_edge_detection

@log_perf
def extract_rois(img_bgr: np.ndarray, roi_info):
    ''' Function to extract ROI for each detected contours in the original image'''
    roi = {}
    for cnt_id in roi_info:
        if not roi_info[cnt_id]:
            continue
        x,y,w,h = roi_info[cnt_id][0]
        roi_img = img_bgr[y:y+h, x:x+w].copy()
        roi[cnt_id]=roi_img
    return roi

@log_perf
def canny_on_roi(roi_img: np.ndarray):
    roi_edges, cleaned_roi_edges = canny_edge_detection(img_bgr=roi_img)
    return roi_edges, cleaned_roi_edges


@log_perf
def line_segment_detection(img_bgr: np.ndarray, full=False):
    ann_img = img_bgr.copy()

    if full:
        edges, cleaned_edges = canny_edge_detection(img_bgr=img_bgr)
    else:
        edges, cleaned_edges = canny_on_roi(roi_img=img_bgr)

    # HoughLinesP is generally better for finding the endpoints of line segments.
    # Parameters:
    # edges: Output of the Canny detector.
    # 1: rho accuracy (distance resolution in pixels)
    # np.pi/180: theta accuracy (angular resolution in radians)
    # 50: threshold (minimum number of intersections to 'detect' a line)
    # 10: minLineLength (minimum line length. Shorter lines are rejected)
    # 5: maxLineGap (maximum allowed gap between line segments to treat them as a single line)
    # tuned to a certain extent only (further tuning would make it better)
    lines = cv2.HoughLinesP(cleaned_edges, 1, np.pi / 180, 34, minLineLength=6, maxLineGap=21)
    print(f"Number of raw lines detected: {len(lines)}")
    if lines is not None:
        for i, line in enumerate(lines):
            x1,y1,x2,y2 = line[0]
            cv2.line(ann_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    else:
        print("No line segments were found.")

    return ann_img


def build_line_matrix(lines):
    ''' Building a Collection of Graphs with line segments as nodes and connections to line as edges'''
    num_lines = len(lines) # number of nodes in the graph
    adjacency_matrix = np.zeros(shape=(len(lines)), dtype=bool)

    



    return adjacency_matrix



