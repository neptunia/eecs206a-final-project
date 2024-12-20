import cv2
import numpy as np
import math


import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import List, Tuple
def animate_points(points_list):
    """
    Animates the drawing of segments between points in the list of lists.
    
    Arguments:
    points_list -- A list of lists, where each inner list contains tuples of (x, y) points
                   to be connected by lines.
    """
    # Set up the plot
    fig, ax = plt.subplots()
    
    # Set axis limits
    ax.set_xlim(0, 1100)  # Adjust limits as necessary
    ax.set_ylim(0, 850)  # Adjust limits as necessary
    
    # Set equal aspect ratio to avoid distortion
    ax.set_aspect('equal')
    
    # Color map for different segments
    total_lists = len(points_list)
    colors = plt.cm.rainbow(np.linspace(0, 1, total_lists))  # One color per list
    
    # Initialize the plot (empty)
    def init():
        return []

    # Function to update the plot at each frame
    def update(frame):
        # Calculate which list of points the current frame corresponds to
        list_idx = frame  # Frame corresponds to the current list being drawn
        total_segments = 0
        lines_to_draw = []

        # Iterate over all the point lists and find the current list
        for points in points_list:
            num_segments = len(points) - 1
            if list_idx == total_segments:
                # Plot the entire list of points with the same color
                x_vals = [point[1] for point in points]
                y_vals = [850-point[0] for point in points]
                
                # Plot the segment with a different color
                ax.plot(x_vals, y_vals, color=colors[list_idx], lw=2)
                break
            total_segments += 1  # Move to the next list

        return lines_to_draw

    # Calculate the total number of lists
    total_lists = len(points_list)

    # Create the animation
    ani = FuncAnimation(fig, update, frames=total_lists+5, init_func=init, blit=False, interval=500, repeat=True, repeat_delay=3000)
    ani.save("animation.mp4", writer="ffmpeg", fps=2)
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
            dist_reversed = calculate_distance(last_end, seg[-1])

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
            if calculate_distance(paths[i][-1], paths[i+1][0]) < 15:
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
    import pickle
    with open("/home/cc/ee106a/fa24/class/ee106a-ads/ros_workspaces/lab7/src/sawyer_full_stack/scripts/data.pickle", "wb") as f:
        pickle.dump(pp, f)
    # Visualize the paths being drawn
    animate_points(pp)
