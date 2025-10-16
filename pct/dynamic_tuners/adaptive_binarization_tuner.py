import cv2
import numpy as np

def nothing(x):
    pass

# Load the image
# NOTE: Using the uploaded file 'test.png' for accessibility in this environment
img = cv2.imread('dataset/samples/0ECFA35F-87BC-4189-8AF8-08DD8B834DD4_Week_168_Visit_PACC_-_Version_A_MMSEdrawing_page1.png')

if img is None:
    print("Error: Image not loaded. Check file path 'test.png'.")
    exit()

# Resize image to fit window
scale_percent = 50  # 50% of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

# Convert to grayscale once
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Create a resizable window and rename it for Adaptive Thresholding
cv2.namedWindow('Adaptive Threshold Tuner', cv2.WINDOW_NORMAL)

# --- CORRECTED TRACKBARS FOR ADAPTIVE THRESHOLDING ---

# 1. Block Size (Must be odd, controls local neighborhood size)
cv2.createTrackbar('Block Size', 'Adaptive Threshold Tuner', 41, 101, nothing) 
# 2. C Value (Constant subtracted from mean)
#    Using a range that includes negative values for flexible tuning.
cv2.createTrackbar('C Value', 'Adaptive Threshold Tuner', 5, 50, nothing) 
cv2.createTrackbar('C Negative', 'Adaptive Threshold Tuner', 0, 1, nothing) # Switch for negative C
cv2.createTrackbar('Kernel Size', 'Adaptive Threshold Tuner', 0, 100, nothing)

# --------------------------------------------------------

while True:
    # Get trackbar positions
    block_size = cv2.getTrackbarPos('Block Size', 'Adaptive Threshold Tuner')
    c_val_mag = cv2.getTrackbarPos('C Value', 'Adaptive Threshold Tuner')
    c_neg_switch = cv2.getTrackbarPos('C Negative', 'Adaptive Threshold Tuner')
    ksize = cv2.getTrackbarPos('Kernel Size', 'Adaptive Threshold Tuner')

    # 1. Sanitize Block Size (Must be odd and >= 3)
    if block_size % 2 == 0:
        block_size += 1
    if block_size < 3:
        block_size = 3

    # 2. Apply C Sign
    c_value = c_val_mag * (-1 if c_neg_switch == 1 else 1)

    # --- Adaptive Mean Thresholding ---
    
    # ADAPTIVE_THRESH_MEAN_C: Threshold is the mean of the block_size neighborhood minus C_value.
    thresh_mean = cv2.adaptiveThreshold(
        gray, 
        255, 
        cv2.ADAPTIVE_THRESH_MEAN_C, 
        cv2.THRESH_BINARY_INV, 
        block_size, 
        c_value
    )

    kernel = np.ones((ksize, ksize), np.uint8) 
    # Apply the morphological opening operation
    cleaned_binary_img = cv2.morphologyEx(thresh_mean, cv2.MORPH_OPEN, kernel)

    # Display the result
    cv2.imshow('Adaptive Threshold Tuner', cleaned_binary_img)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC key
        break

cv2.destroyAllWindows()