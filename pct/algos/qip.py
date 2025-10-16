import numpy as np
import cv2
from collections import deque
from typing import Any

from utils import canny_edge_detection, get_point_distance, approximate_polygon

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

def canny_on_roi(roi_img: np.ndarray):
    ''' Function to perform canny edge detection on the extracted ROIs'''
    roi_edges, cleaned_roi_edges = canny_edge_detection(img_bgr=roi_img)
    return roi_edges, cleaned_roi_edges

def line_segment_detection(img_bgr: np.ndarray, full=False, annotate=False):
    ''' Houghlines Probablistic function implemented to find the line segments on the detected edges '''
    interim_ann = img_bgr.copy()
    if full:
        edges, cleaned_edges = canny_edge_detection(img_bgr=img_bgr)
    else:
        edges, cleaned_edges = canny_on_roi(roi_img=img_bgr)

    # tuned to a certain extent only (further tuning would make it better)
    lines = cv2.HoughLinesP(cleaned_edges, 1, np.pi / 180, 34, minLineLength=6, maxLineGap=21)
    # print(f"Number of raw lines detected: {len(lines)}")
    if annotate:
        if lines is not None:
            for i, line in enumerate(lines):
                x1,y1,x2,y2 = line[0]
                cv2.line(interim_ann, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # plotting line segment end points
                cv2.circle(interim_ann, (x1, y1), 5, (0,0,0), 1)
        else:
            print("No line segments were found.")

        return interim_ann, lines

    return None, lines

def build_line_matrix(lines, config: dict):
    ''' Building a Collection of Graphs with line segments as nodes and connections to line as edges'''

    num_lines = len(lines) # number of nodes in the graph
    adjacency_matrix = np.zeros(shape=(num_lines, num_lines), dtype=bool)

    # ---------------- TUNABLE PARAMS ----------------
    CONNECTION_TOLERANCE = config["CONNECTION_TOLERANCE"] # 5 # pixel distance
    # ---------------------------------------
    for i in range(num_lines):
        for j in range(i+1, num_lines):
            x1,y1,x2,y2 = lines[i][0]
            x3,y3,x4,y4 = lines[j][0]
            connected = False # init flag for checking connections

            # an extra condition to ignore intersection tolerance can be added to make this better (later on)
          
            endpoints_i = [(x1,y1), (x2,y2)]
            endpoints_j = [(x3,y3), (x4,y4)]
            for p in endpoints_i:
                for q in endpoints_j:
                    dist = get_point_distance(np.array(p), np.array(q))
                    if dist < CONNECTION_TOLERANCE: # implies the line segments are connected
                        connected = True
                        break
                if connected:
                    break
            if connected:
                adjacency_matrix[i, j] = adjacency_matrix[j, i] = True

    return adjacency_matrix

def get_clusters(lines, config: dict):
    ''' Unravelling the interlocked pentagons to extract collection points lying on the pentagons which will basically be the endpoints'''    
    conn_matrix = build_line_matrix(lines=lines, config=config)
    num_lines = conn_matrix.shape[0]
    visited_nodes = np.zeros(shape=(num_lines), dtype=bool)
    clusters = []
    # cluster extraction algo "BFS graph search algorithm"
    for i in range(num_lines):
        if not visited_nodes[i]:
            queue = deque([i])
            cluster = []
            while queue:
                current_node = queue.popleft()
                visited_nodes[current_node] = True
                cluster.append(current_node)
                neighbours = np.where(conn_matrix[current_node]==1)[0]
                for neigh_node in neighbours:
                    if not visited_nodes[neigh_node]:
                        queue.append(neigh_node)
    
            clusters.append(cluster)

    # print(f"Number of clusters extracted: {len(clusters)}")
    return clusters

def cluster_to_contour(lines: np.ndarray, cluster: list):
    ''' converting the unravelled clusters to convex hull contours (automatically orders the points no need to perform circle sort here) '''

    # cluster list of collection of indices representing each detected line segments of the polygon
    collected_points = []
    for line_idx in cluster:
        x1,y1,x2,y2 = lines[line_idx][0]
        collected_points.extend([(x1,y1), (x2,y2)])
    
    # print(f"Number of points in one cluster: {len(collected_points)}")
    # points to contours
    hull = cv2.convexHull(np.array(collected_points), returnPoints=True) # gives and enclosing convex polygon over these points
    return hull, collected_points

def extract_polygon(img_bgr: np.ndarray, config: dict, hull: Any, annotate = False):
    ''' Approximating a polygon over the extracted cluster->hull contours gives out the polygon details'''
    poly_approx, num_vertices, vertices = approximate_polygon(contour=hull, config=config)
    if annotate:
        # annotate the approximated polygon outline and vertex points
        cv2.drawContours(img_bgr, [poly_approx], -1, (255, 0, 0), 1)
        for i, vertex in enumerate(vertices):
            vx,vy = vertex
            cv2.circle(img_bgr, (vx, vy), 6, (0, 0, 255), -1)
            cv2.putText(img_bgr, f"v-{i}", (vx,vy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)
        return img_bgr, vertices
    
    return None, poly_approx, vertices
