import cv2
import numpy as np


def binarize_img(img_bgr: np.ndarray):
    ''' Function basically does the canny edge detection to detect the figures and features in the input image'''
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


def nothing(x):
    pass

# Load image
img = cv2.imread('dataset/samples/0ECFA35F-87BC-4189-8AF8-08DD8B834DD4_Week_168_Visit_PACC_-_Version_A_MMSEdrawing_page1.png')
edges, cleaned_edges = binarize_img(img_bgr=img)

# Create window
cv2.namedWindow('HoughLines Combined Tuner', cv2.WINDOW_NORMAL)

# Trackbars
cv2.createTrackbar('Method', 'HoughLines Combined Tuner', 0, 1, nothing)       # 0: HoughLines, 1: HoughLinesP
cv2.createTrackbar('Threshold', 'HoughLines Combined Tuner', 150, 500, nothing)
cv2.createTrackbar('Rho', 'HoughLines Combined Tuner', 1, 10, nothing)
cv2.createTrackbar('Theta', 'HoughLines Combined Tuner', 180, 360, nothing)
cv2.createTrackbar('MinLineLength', 'HoughLines Combined Tuner', 50, 500, nothing)
cv2.createTrackbar('MaxLineGap', 'HoughLines Combined Tuner', 10, 100, nothing)

while True:
    method = cv2.getTrackbarPos('Method', 'HoughLines Combined Tuner')
    threshold = cv2.getTrackbarPos('Threshold', 'HoughLines Combined Tuner')
    rho = cv2.getTrackbarPos('Rho', 'HoughLines Combined Tuner')
    theta = cv2.getTrackbarPos('Theta', 'HoughLines Combined Tuner')
    theta_rad = np.pi / theta if theta != 0 else np.pi / 180

    minLineLength = cv2.getTrackbarPos('MinLineLength', 'HoughLines Combined Tuner')
    maxLineGap = cv2.getTrackbarPos('MaxLineGap', 'HoughLines Combined Tuner')

    img_copy = img.copy()

    if method == 0:
        # HoughLines
        if threshold > 0:
            lines = cv2.HoughLines(cleaned_edges, rho, theta_rad, threshold)
            if lines is not None:
                for line in lines:
                    rho_l, theta_l = line[0]
                    a = np.cos(theta_l)
                    b = np.sin(theta_l)
                    x0 = a * rho_l
                    y0 = b * rho_l
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))
                    cv2.line(img_copy, (x1, y1), (x2, y2), (0, 0, 255), 2)
    else:
        # HoughLinesP
        if threshold > 0:
            lines = cv2.HoughLinesP(edges, rho, theta_rad, threshold, minLineLength=minLineLength, maxLineGap=maxLineGap)
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(img_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.imshow('HoughLines Combined Tuner', img_copy)

    key = cv2.waitKey(100) & 0xFF
    if key == 27:  # ESC
        break

cv2.destroyAllWindows()
