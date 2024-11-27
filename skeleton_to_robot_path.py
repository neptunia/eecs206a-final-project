import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.cm import get_cmap
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
    animation_steps = []

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
            animation_steps.append(list(visited))
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
    
    return paths, animation_steps


# Animation function
def animate_paths(image, animation_steps):
    fig, ax = plt.subplots()
    ax.imshow(image, cmap='gray')
    ax.set_title("Colored Path Visualization")
    
    # Set up colors
    cmap = get_cmap("tab10")  # Choose a colormap
    num_paths = len(paths)
    colors = [cmap(i / num_paths) for i in range(num_paths)]

    # Path data for animation
    x_data, y_data, color_data = [], [], []

    for path, color in zip(paths, colors):
        x, y = zip(*path)
        x_data.append(x)
        y_data.append(y)
        color_data.append(color)

    points = []

    def init():
        for color in color_data:
            point, = ax.plot([], [], '.', color=color, markersize=2)
            points.append(point)
        return points

    def update(frame):
        for i, point in enumerate(points[:frame + 1]):
            point.set_data(y_data[i], x_data[i])
        return points

    ani = FuncAnimation(
        fig, update, frames=num_paths, init_func=init, interval=500, repeat=False
    )
    plt.show()

# Main script
if __name__ == "__main__":
    # Load the image in grayscale
    image = cv2.imread("result.png", cv2.IMREAD_GRAYSCALE)

    # Threshold to ensure binary image
    _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

    # Extract paths and animation steps
    paths, animation_steps = extract_paths_prioritize_dynamic_endpoints(binary_image)
    for i in range(len(paths)):
        print("Path %i: "%i, paths[i])
    # Visualize the paths being drawn
    animate_paths(binary_image, animation_steps)