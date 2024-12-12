#! /usr/bin/env python
# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface


def show_image_callback(img_data, xxx_todo_changeme):
    """The callback function to show image by using CvBridge and cv
    """
    (edge_detection, window_name) = xxx_todo_changeme
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError as err:
        rospy.logerr(err)
        return
    if edge_detection == True:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        # customize the second and the third argument, minVal and maxVal
        # in function cv2.Canny if needed
        get_edge = cv2.Canny(blurred, 10, 100)
        cv_image = np.hstack([get_edge])
    edge_str = "(Edge Detection)" if edge_detection else ''
    cv_win_name = ' '.join([window_name, edge_str])
    cv2.imwrite("./test.png",cv_image)

def main():
    """Camera Display Example

    Cognex Hand Camera Ranges
        - exposure: [0.01-100]
        - gain: [0-255]
    Head Camera Ranges:
        - exposure: [0-100], -1 for auto-exposure
        - gain: [0-79], -1 for auto-gain
    """
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
            " parameters on this robot. Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-c', '--camera', type=str, default="head_camera",
        choices=valid_cameras, help='Setup Camera Name for Camera Display')
    parser.add_argument(
        '-r', '--raw', action='store_true',
        help='Specify use of the raw image (unrectified) topic')
    parser.add_argument(
        '-e', '--edge', action='store_true',
        help='Streaming the Canny edge detection image')
    parser.add_argument(
        '-g', '--gain', type=int,
        help='Set gain for camera (-1 = auto)')
    parser.add_argument(
        '-x', '--exposure', type=float,
        help='Set exposure for camera (-1 = auto)')
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node('camera_display', anonymous=True)
    cameras = intera_interface.Cameras()
    if not cameras.verify_camera_exists(args.camera):
        rospy.logerr("Could not detect the specified camera, exiting the example.")
        return
    rospy.loginfo("Opening camera '{0}'...".format(args.camera))
    cameras.start_streaming(args.camera)
    rectify_image = not args.raw
    use_canny_edge = args.edge
    cameras.set_callback(args.camera, show_image_callback,
        rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))

    # optionally set gain and exposure parameters
    if args.gain is not None:
        if cameras.set_gain(args.camera, args.gain):
            rospy.loginfo("Gain set to: {0}".format(cameras.get_gain(args.camera)))

    if args.exposure is not None:
        if cameras.set_exposure(args.camera, args.exposure):
            rospy.loginfo("Exposure set to: {0}".format(cameras.get_exposure(args.camera)))

    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.sleep(1)
    rospy.signal_shutdown("done")

main()




# Load the image
image_path = "test.png"  # Replace with your image file path
image = cv2.imread(image_path)

# Convert to HSV color space for better color segmentation
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the color range for the paper (adjust based on your image)
# Replace these values with appropriate HSV ranges for the paper's color
lower_color = np.array([0, 0, 170])  # Example: white paper
upper_color = np.array([180, 60, 255])

# Create a mask for the color range
mask = cv2.inRange(hsv, lower_color, upper_color)

# Perform morphological operations to clean up the mask
kernel = np.ones((5, 5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Close small holes
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Remove small noise

# Apply the mask to the original image
masked_image = cv2.bitwise_and(image, image, mask=mask)

# Find the largest connected region to isolate the paper
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if contours:
    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Create a new mask for the largest region
    largest_mask = np.zeros_like(mask)
    cv2.drawContours(largest_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)
    
    # Mask the original image with the largest region
    final_masked_image = cv2.bitwise_and(image, image, mask=largest_mask)
else:
    print("No uniform color region detected.")
    final_masked_image = image


# Apply morphological operations to clean the mask
kernel = np.ones((5, 5), np.uint8)  # Define the kernel size (adjust as needed)

cleaned_mask = cv2.erode(largest_mask,kernel,iterations = 30)
cleaned_mask = cv2.dilate(cleaned_mask,kernel,iterations = 30)


contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if contours:
    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Create a new mask for the largest region
    largest_mask = np.zeros_like(cleaned_mask)
    cv2.drawContours(largest_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)
    
    # Mask the original image with the largest region
    final_masked_image = cv2.bitwise_and(image, image, mask=largest_mask)
else:
    print("No uniform color region detected.")
    final_masked_image = image

cv2.imwrite("masked_image.jpg", final_masked_image)


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

from skimage.morphology import skeletonize

# Load the image
image_path = 'transformed_image.png'
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
image = ~image

(thresh, im_bw) = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
im_bw = im_bw/255
# skeletonize
skeleton = skeletonize(im_bw).astype(np.uint8)

binary = skeleton*255

print(binary)


# Create a mask to find connected regions
mask = np.zeros((binary.shape[0] + 2, binary.shape[1] + 2), dtype=np.uint8)

# Flood fill from the edges to detect edge-connected contours (using 8-connectivity)
for y in range(binary.shape[0]):
    if binary[y, 0] == 255:  # Left edge
        cv2.floodFill(binary, mask, (0, y), 0, flags=cv2.FLOODFILL_MASK_ONLY | cv2.FLOODFILL_FIXED_RANGE | 8)
    if binary[y, binary.shape[1] - 1] == 255:  # Right edge
        cv2.floodFill(binary, mask, (binary.shape[1] - 1, y), 0, flags=cv2.FLOODFILL_MASK_ONLY | cv2.FLOODFILL_FIXED_RANGE | 8)

for x in range(binary.shape[1]):
    if binary[0, x] == 255:  # Top edge
        cv2.floodFill(binary, mask, (x, 0), 0, flags=cv2.FLOODFILL_MASK_ONLY | cv2.FLOODFILL_FIXED_RANGE | 8)
    if binary[binary.shape[0] - 1, x] == 255:  # Bottom edge
        cv2.floodFill(binary, mask, (x, binary.shape[0] - 1), 0, flags=cv2.FLOODFILL_MASK_ONLY | cv2.FLOODFILL_FIXED_RANGE | 8)

# Mask is now filled with 1s where edge-connected regions were found
# Invert the mask and apply it to the binary image to remove those regions
binary[mask[1:-1, 1:-1] == 1] = 0

kernel = np.ones((5, 5), np.uint8)


# Count the number of white neighbors for each pixel using a convolution
neighbor_count = cv2.filter2D(binary // 255, -1, kernel)

# Remove isolated white pixels (those with no other white neighbors)
binary[(binary == 255) & (neighbor_count == 1)] = 0
h, w = binary.shape
binary[0:100, 0:100] = 0  # Top-left corner
binary[0:100, w-100:w] = 0  # Top-right corner
binary[h-100:h, 0:100] = 0  # Bottom-left corner
binary[h-100:h, w-100:w] = 0  # Bottom-right corner

# Save
cv2.imwrite("result.png", binary)

import math


import matplotlib.pyplot as plt
import matplotlib.animation as animation
from typing import List, Tuple

def animate_paths(paths: List[List[Tuple[int, int]]], save_as: str = None):
    """
    Create an animation of paths being drawn.
    
    :param paths: List of paths, where each path is a list of points [(x1, y1), (x2, y2), ...].
    :param save_as: Optional filename to save the animation (e.g., 'animation.mp4').
    """
    # Set up the figure and axis
    fig, ax = plt.subplots(figsize=(10, 10))  # 1000x1000 pixels
    ax.set_xlim(0, 1000)  # Adjust according to the data range
    ax.set_ylim(0, 1000)
    ax.set_aspect('equal')
    
    # Lines for each path
    lines = [ax.plot([], [], lw=2)[0] for _ in paths]
    
    # Create a list of cumulative points for each path
    cumulative_points = [[] for _ in paths]

    def init():
        """Initialize the animation."""
        for line in lines:
            line.set_data([], [])
        return lines

    def update(frame):
        """Update the frame of the animation."""
        for i, path in enumerate(paths):
            if frame < len(path) - 1:  # Ensure we don't go out of bounds
                # Append the current segment to the cumulative points for this path
                cumulative_points[i].append(path[frame + 1])
                # Update the line with cumulative points
                x_data, y_data = zip(*cumulative_points[i])
                lines[i].set_data(x_data, y_data)
        return lines

    # Total frames = longest path length minus 1 (since we're drawing line segments)
    max_frames = max(len(path) - 1 for path in paths)

    # Create the animation
    anim = animation.FuncAnimation(
        fig, update, frames=max_frames, init_func=init, blit=True, interval=5, repeat=False
    )

    # Save the animation if a filename is provided
    if save_as:
        anim.save(save_as, fps=30, extra_args=['-vcodec', 'libx264'])

    plt.show()

def extract_paths_prioritize_dynamic_endpoints(image):
    """
    Extracts paths from a skeletonized binary image using DFS, dynamically prioritizing endpoint pixels.
    
    Args:
        image (np.ndarray): Binary image with skeletonized structure (white: 255, black: 0).

    Returns:
        list: A list of paths, where each path is a list of (x, y) tuples representing pixel coordinates.
    """
    # Get the coordinates of all white pixels
    white_pixels = np.column_stack(np.where(image == 255))
    
    # Map white pixel coordinates to their neighbors
    white_pixel_set = set(map(tuple, white_pixels))
    
    # To track visited pixels
    visited = set()

    # List to store all paths
    paths = []

    # Neighbor offsets
    neighbor_offsets = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    def get_neighbors(pixel):
        """Return unvisited neighbors of a pixel."""
        x, y = pixel
        return [(x + dx, y + dy) for dx, dy in neighbor_offsets if (x + dx, y + dy) in white_pixel_set and (x + dx, y + dy) not in visited]

    def find_endpoint():
        """Find an endpoint (pixel with only one unvisited neighbor) or return any unvisited pixel."""
        for pixel in white_pixel_set:
            if pixel not in visited:
                if len(get_neighbors(pixel)) == 1:
                    return pixel
        # If no endpoint, return any unvisited pixel
        return next((pixel for pixel in white_pixel_set if pixel not in visited), None)

    # Depth-first search
    def dfs(pixel):
        stack = [pixel]
        path = []
        
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            path.append(current)
            # Choose a single unvisited neighbor to follow
            neighbors = get_neighbors(current)
            if neighbors:
                stack.append(neighbors[0])  # Follow the first unvisited neighbor
        return path

    # Traverse all white pixels
    while True:
        start_pixel = find_endpoint()
        if not start_pixel:  # No unvisited pixels left
            break
        path = dfs(start_pixel)
        if path:  # Add non-empty paths
            paths.append(path)
    
    return paths

def calculate_distance(p1, p2) -> float:
    """Calculate Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[-1] - p2[-1])**2)

def order_segments(
    segments: List[Tuple[Tuple[float, float], Tuple[float, float]]]
) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """
    Order line segments to minimize distance between consecutive segments, allowing reversal of segments.
    
    :param segments: List of line segments as [(start, end), ...].
    :return: Ordered and possibly reversed list of segments.
    """
    if not segments:
        return []

    # Start with the first segment
    ordered_segments = [segments.pop(0)]

    while segments:
        # Get the last segment's endpoint
        last_end = ordered_segments[-1][-1]

        # Find the closest segment (considering both orientations)
        best_segment = None
        best_distance = float('inf')

        for seg in segments:
            # Distance without reversal
            dist_normal = calculate_distance(last_end, seg[0])
            # Distance with reversal
            dist_reversed = calculate_distance(last_end, seg[1])

            if dist_normal < best_distance:
                best_distance = dist_normal
                best_segment = (seg, False)  # False means no reversal

            if dist_reversed < best_distance:
                best_distance = dist_reversed
                best_segment = (seg, True)  # True means reversed

        # Add the best segment to the ordered list
        segment, reversed_flag = best_segment
        if reversed_flag:
            ordered_segments.append(segment[::-1])  # Reverse the segment
        else:
            ordered_segments.append(segment)

        # Remove the selected segment from the pool
        segments.remove(segment)

    return ordered_segments

def preprocess_paths(raw_paths: List[np.ndarray]) -> List[List[Tuple[int, int]]]:
    """
    Flatten raw paths with extra dimensionality into a simpler form.
    
    :param raw_paths: List of np.ndarrays with shape (n, 1, 2).
    :return: List of paths as [(x1, y1), (x2, y2), ...].
    """
    return [[tuple(point[0]) for point in path] for path in raw_paths]

# Main script
if __name__ == "__main__":
    # Load the image in grayscale
    image = cv2.imread("result.png", cv2.IMREAD_GRAYSCALE)

    # Threshold to ensure binary image
    _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

    # Extract paths and animation steps
    paths = extract_paths_prioritize_dynamic_endpoints(binary_image)

    # drop paths with less than 5 points
    paths = [i for i in paths if len(i) > 4]

    for i in range(10):
    
        paths = order_segments(paths)

        # merge paths if the end points are within 10 pixels of each other

        for i in range(len(paths)-2, 0, -1):
            if calculate_distance(paths[i][-1], paths[i+1][0]) < 10:
                paths[i]+=paths[i+1]
                paths.pop(i+1)
        #print(paths_reordered)
        #for i in range(len(paths_reordered)):
        #    print("Path %i: "%i, paths_reordered[i])

    paths_reordered = order_segments(paths)

    # drop paths with less than 20 points
    paths_reordered = [i for i in paths_reordered if len(i) > 20]


    newpaths = []
    eps = 0.005
    for path in paths_reordered:
        print(path)
        path = np.array(path)
        peri = cv2.arcLength(path, True)
        approx = cv2.approxPolyDP(path, eps*peri, False)
        #approx = [path[0]] + approx + [path[-1]]
        #print([i[0] for i in approx])
        #print(approx)
        newpaths.append(approx)
    #print(newpaths)


    pp = preprocess_paths(newpaths)
    print(pp)
    # Visualize the paths being drawn
    animate_paths(pp)
