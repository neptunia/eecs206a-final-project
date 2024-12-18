import cv2
import numpy as np
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
        #print(path)
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
