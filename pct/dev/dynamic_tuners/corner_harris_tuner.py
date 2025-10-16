import cv2
import numpy as np

def nothing(x):
    pass

# Load image
img = cv2.imread('your_image.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Create window
cv2.namedWindow('Harris Corner Tuner', cv2.WINDOW_NORMAL)

# Create trackbars for blockSize, ksize, and k
cv2.createTrackbar('blockSize', 'Harris Corner Tuner', 2, 10, nothing)  # Neighborhood size
cv2.createTrackbar('ksize', 'Harris Corner Tuner', 3, 31, nothing)      # Aperture size (must be odd)
cv2.createTrackbar('k*100', 'Harris Corner Tuner', 4, 10, nothing)      # k parameter scaled by 100

while True:
    blockSize = cv2.getTrackbarPos('blockSize', 'Harris Corner Tuner')
    ksize = cv2.getTrackbarPos('ksize', 'Harris Corner Tuner')
    k = cv2.getTrackbarPos('k*100', 'Harris Corner Tuner') / 100.0

    # Ensure valid parameters
    blockSize = max(2, blockSize)  # must be >=2
    if ksize % 2 == 0:  # ksize must be odd and >=3
        ksize += 1
    ksize = max(3, ksize)

    # Harris corner detection
    gray_float = np.float32(gray)
    dst = cv2.cornerHarris(gray_float, blockSize, ksize, k)
    dst = cv2.dilate(dst, None)

    # Create a copy to draw corners
    display_img = img.copy()
    display_img[dst > 0.01 * dst.max()] = [0, 0, 255]

    cv2.imshow('Harris Corner Tuner', display_img)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC key to exit
        break

cv2.destroyAllWindows()
