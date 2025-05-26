import cv2
import numpy as np
import os
import sys

# --- Configuration ---
image_filename = 'frame.jpeg'  # **Ensure this matches your filename**
image_path = image_filename  # Path to the image

# Canny thresholds for different experiments
threshold_pairs = [
    (50, 150),
    (75, 200),
    (100, 250)
]

# Hough Transform parameters
rho = 1           # Distance resolution in pixels.
theta = np.pi / 180  # Angle resolution in radians.
minLineLength = 30  # Minimum line length
maxLineGap = 10    # Maximum allowed gap between line segments

# HSV color range for yellow lane detection
lower_yellow_hsv = np.array([15, 80, 80])
upper_yellow_hsv = np.array([35, 255, 255])

# RGB color range for yellow lane detection
lower_yellow_rgb = np.array([0, 100, 100])
upper_yellow_rgb = np.array([100, 255, 255])

# --- Function Definitions ---

# Canny edge detection experiment
def run_canny_experiment(img, threshold_pairs):
    for i, (low_threshold, high_threshold) in enumerate(threshold_pairs):
        edges = cv2.Canny(img, low_threshold, high_threshold)
        cv2.imshow(f'Canny Edge Detection - Low: {low_threshold}, High: {high_threshold}', edges)

# Hough Transform experiment
def run_hough_transform(img, edges, minLineLength, maxLineGap, threshold):
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, minLineLength=minLineLength, maxLineGap=maxLineGap)
    line_img = np.copy(img) * 0  # Blank image for drawing lines
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    combined_img = cv2.addWeighted(img, 0.8, line_img, 1, 0)
    cv2.imshow(f'Hough Transform (Threshold: {threshold})', combined_img)

# Compare HSV and RGB for yellow detection
def compare_hsv_rgb(img):
    # Convert image to HSV and RGB
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Apply yellow detection in HSV
    yellow_mask_hsv = cv2.inRange(hsv_img, lower_yellow_hsv, upper_yellow_hsv)
    yellow_filtered_color_hsv = cv2.bitwise_and(img, img, mask=yellow_mask_hsv)

    # Apply yellow detection in RGB
    yellow_mask_rgb = cv2.inRange(rgb_img, lower_yellow_rgb, upper_yellow_rgb)
    yellow_filtered_color_rgb = cv2.bitwise_and(img, img, mask=yellow_mask_rgb)

    # Display results
    cv2.imshow('Yellow Detection (HSV)', yellow_filtered_color_hsv)
    cv2.imshow('Yellow Detection (RGB)', yellow_filtered_color_rgb)

# Simulate lighting conditions (brightness and darkness)
def simulate_lighting_conditions(img):
    # Simulate brighter and darker images
    alpha_bright = 1.5  # Brightness factor
    beta_bright = 50    # Brightness offset
    alpha_dark = 0.6    # Darker factor
    beta_dark = 0       # Darker offset

    img_bright = cv2.convertScaleAbs(img, alpha=alpha_bright, beta=beta_bright)
    img_dark = cv2.convertScaleAbs(img, alpha=alpha_dark, beta=beta_dark)

    # Show the simulated lighting conditions
    cv2.imshow('Brighter Image', img_bright)
    cv2.imshow('Darker Image', img_dark)

# --- Main Execution ---

if __name__ == "__main__":
    # Load the original image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not load image from {image_path}")
        sys.exit(1)
    print(f"Successfully loaded image: {image_path}")

    # Convert image to grayscale for edge detection experiments
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Experiment 1: Run Canny Edge Detection with three different threshold pairs
    run_canny_experiment(gray_img, threshold_pairs)

    # Experiment 2: Apply Hough Transform with varying threshold and observe effect
    for threshold in [50, 100, 150]:  # Varying the Hough Transform threshold
        run_hough_transform(img, gray_img, minLineLength, maxLineGap, threshold)

    # Experiment 3: Compare HSV vs RGB for yellow lane detection
    compare_hsv_rgb(img)

    # Experiment 4: Simulate different lighting conditions
    simulate_lighting_conditions(img)

    print("Experiments completed. Press any key on one of the image windows to close them.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
