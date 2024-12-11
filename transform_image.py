import cv2
import numpy as np

# Load the masked image (assumes the background is black, and the paper is white)
masked_image = cv2.imread("masked_image.jpg")
gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

# Threshold to ensure binary mask
_, binary_mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)


contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find the largest contour (assumes the paper is the largest object)
if contours:
    largest_contour = max(contours, key=cv2.contourArea)
else:
    print("No contours found in the mask!")
    exit()

hull = cv2.convexHull(largest_contour)

# Approximate the convex hull as a quadrilateral
epsilon = 0.02 * cv2.arcLength(hull, True)  # Adjust epsilon as needed
approx = cv2.approxPolyDP(hull, epsilon, True)

# If we don't get exactly 4 corners, force a quadrilateral
if len(approx) < 4:
    print(f"Only {len(approx)} points detected; forcing quadrilateral.")
    rect = cv2.boundingRect(hull)
    x, y, w, h = rect
    approx = np.array([
        [x, y],
        [x + w, y],
        [x + w, y + h],
        [x, y + h]
    ]).reshape(-1, 1, 2)

# Ensure we have exactly 4 points for perspective transform
points = approx.reshape(-1, 2)
if len(points) > 4:
    # Keep only the most extreme points (convex hull ensures correctness)
    points = cv2.convexHull(points, returnPoints=True).squeeze()

# Order the points to ensure a consistent mapping
def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # Top-left
    rect[2] = pts[np.argmax(s)]  # Bottom-right
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # Top-right
    rect[3] = pts[np.argmax(diff)]  # Bottom-left
    return rect

ordered_points = order_points(points)

# Define destination points for the 1000x1000 square
dst = np.array([
    [0, 0],
    [1100, 0],
    [1100, 850],
    [0, 850]
], dtype="float32")

# Compute the perspective transformation matrix
matrix = cv2.getPerspectiveTransform(ordered_points, dst)

# Perform the perspective transformation
transformed_image = cv2.warpPerspective(masked_image, matrix, (1100, 850), borderValue=(255, 255, 255))


# Save and display the transformed image
cv2.imshow("Transformed Image", transformed_image)
cv2.imwrite("transformed_image.png", transformed_image)
cv2.waitKey(0)
cv2.destroyAllWindows()