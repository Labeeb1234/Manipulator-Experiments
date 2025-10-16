import cv2
import numpy as np

def nothing(x):
    pass

# Load the image
img = cv2.imread('dataset/6B0D6B92-2DE3-42F1-8AF7-08DD8B834DD4_Week_168_Visit_PACC_-_Version_A_MMSEdrawing_page1.png')

# Resize image to fit window
scale_percent = 50  # 50% of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

# Create a resizable window
cv2.namedWindow('Canny Edge Tuner', cv2.WINDOW_NORMAL)

# Create trackbars for parameters
cv2.createTrackbar('Threshold1', 'Canny Edge Tuner', 100, 500, nothing)
cv2.createTrackbar('Threshold2', 'Canny Edge Tuner', 200, 500, nothing)
cv2.createTrackbar('Blur Kernel', 'Canny Edge Tuner', 5, 20, nothing)
cv2.createTrackbar('SigmaX', 'Canny Edge Tuner', 14, 50, nothing)  # SigmaX scaled by 10

while True:
    t1 = cv2.getTrackbarPos('Threshold1', 'Canny Edge Tuner')
    t2 = cv2.getTrackbarPos('Threshold2', 'Canny Edge Tuner')
    blur_k = cv2.getTrackbarPos('Blur Kernel', 'Canny Edge Tuner')
    sigma = cv2.getTrackbarPos('SigmaX', 'Canny Edge Tuner') / 10.0

    if blur_k % 2 == 0:
        blur_k += 1
    if blur_k < 3:
        blur_k = 3

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_k, blur_k), sigma)
    edges = cv2.Canny(blurred, t1, t2)

    cv2.imshow('Canny Edge Tuner', edges)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC key
        break

cv2.destroyAllWindows()
