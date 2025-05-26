#!/usr/bin/env python3

import cv2
import numpy as np

# Load image
image = cv2.imread('frame.jpeg')
resized = cv2.resize(image, (640, 480))

# 1. Canny Edge Detection with three different thresholds
def apply_canny(img, low, high):
    edges = cv2.Canny(img, low, high)
    return edges

# 2. Hough Line Transform
def apply_hough(img, threshold=50, min_line_length=50, max_line_gap=10):
    lines = cv2.HoughLinesP(img, 1, np.pi / 180, threshold, minLineLength=min_line_length, maxLineGap=max_line_gap)
    line_img = np.copy(img)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1), (x2, y2), 255, 2)
    return line_img

# 3. Yellow lane detection in RGB and HSV
def detect_yellow_rgb(img):
    lower = np.array([200, 200, 0])
    upper = np.array([255, 255, 150])
    mask = cv2.inRange(img, lower, upper)
    return cv2.bitwise_and(img, img, mask=mask)

def detect_yellow_hsv(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([20, 100, 100])
    upper = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    return cv2.bitwise_and(img, img, mask=mask)

# 4. Simulate lighting change
def adjust_brightness(img, factor):
    return cv2.convertScaleAbs(img, alpha=factor, beta=0)

# ---------- Run and Save Outputs ----------
cv2.imwrite("canny_1.png", apply_canny(resized, 50, 150))
cv2.imwrite("canny_2.png", apply_canny(resized, 100, 200))
cv2.imwrite("canny_3.png", apply_canny(resized, 150, 250))

canny_output = apply_canny(resized, 100, 200)
cv2.imwrite("hough_1.png", apply_hough(canny_output, threshold=30))
cv2.imwrite("hough_2.png", apply_hough(canny_output, threshold=50))
cv2.imwrite("hough_3.png", apply_hough(canny_output, threshold=80))

cv2.imwrite("yellow_rgb.png", detect_yellow_rgb(resized))
cv2.imwrite("yellow_hsv.png", detect_yellow_hsv(resized))

bright = adjust_brightness(resized, 1.5)
dark = adjust_brightness(resized, 0.5)
cv2.imwrite("bright_lane.png", detect_yellow_hsv(bright))
cv2.imwrite("dark_lane.png", detect_yellow_hsv(dark))
