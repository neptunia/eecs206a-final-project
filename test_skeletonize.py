import cv2
import numpy as np
from skimage.morphology import skeletonize
from skimage.util import invert
from skimage.measure import label, regionprops
import networkx as nx

def process_image_to_single_pixel_lines(image_path, output_path):
    # Read the image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    #image = ~image

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(image, (5, 5), 0)

    cv2.imshow("Morphologically Cleaned Maskasdf", blurred)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Use adaptive thresholding for better line extraction
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 13, 2)
    



    # Ensure the image is binary
    binary = (thresh > 0).astype(np.uint8)

    kernel = np.ones((5,5),np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    cv2.imshow("Morphologically Cleaned Mask", binary*255)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    '''
    # Connected component analysis to identify distinct line segments
    labeled_image = label(binary, connectivity=2)
    regions = regionprops(labeled_image)

    # Initialize a graph to store line structure
    G = nx.Graph()

    for region in regions:
        # Skip small noise components
        if region.area < 10:
            continue

        # Approximate the region's centroid and bounding box as nodes
        centroid = tuple(map(int, region.centroid))
        bbox = region.bbox

        # Add nodes and edges based on region properties
        G.add_node(centroid)
        for other in regions:
            if other == region or other.area < 10:
                continue
            other_centroid = tuple(map(int, other.centroid))
            if np.linalg.norm(np.array(centroid) - np.array(other_centroid)) < 20:
                G.add_edge(centroid, other_centroid)
    '''
    # Refine binary image using skeletonization, preserving connectivity
    skeleton = skeletonize(binary.astype(bool)).astype(np.uint8)

    binary = skeleton * 255

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


    # Convert skeleton back to uint8 format for saving
    skeleton_uint8 = (binary).astype(np.uint8)

    return skeleton_uint8


def remove_small_line_noise(binary_image, min_size):
    """
    Removes small noise components (connected lines or clusters) smaller than a specified size.
    
    Args:
        binary_image (numpy.ndarray): Input binary image.
        min_size (int): Minimum size (in pixels) of line segments to keep.
        
    Returns:
        numpy.ndarray: Binary image with noise removed.
    """
    labeled_image, num_features = label(binary_image, connectivity=2, return_num=True)
    cleaned_binary = np.zeros_like(binary_image)

    for region in regionprops(labeled_image):
        # Measure the length of the region (bounding box diagonal, for example)
        min_row, min_col, max_row, max_col = region.bbox
        bbox_diagonal = np.sqrt((max_row - min_row)**2 + (max_col - min_col)**2)
        
        # Keep regions larger than the minimum size
        if bbox_diagonal > min_size:
            cleaned_binary[labeled_image == region.label] = 255

    return cleaned_binary

from skimage.morphology import thin
import numpy as np
from scipy import ndimage
from skimage import measure
import matplotlib.pyplot as plt
def get_neighbors(x, y, shape):
    """Get 8-connectivity neighbors for pixel (x, y)."""
    neighbors = [
        (x-1, y-1), (x-1, y), (x-1, y+1),
        (x, y-1),              (x, y+1),
        (x+1, y-1), (x+1, y), (x+1, y+1)
    ]
    return [(nx, ny) for nx, ny in neighbors if 0 <= nx < shape[0] and 0 <= ny < shape[1]]

def find_tips(image):
    """Find all tips (pixels with exactly one white neighbor)."""
    tips = []
    # Iterate over the image to find tips
    for x in range(image.shape[0]):
        for y in range(image.shape[1]):
            if image[x, y] > 0:
                # Get neighbors of the white pixel
                neighbors = get_neighbors(x, y, image.shape)
                # Count how many white neighbors the pixel has
                white_neighbors = [(nx, ny) for nx, ny in neighbors if image[nx, ny] > 0]
                
                # If the pixel has exactly one white neighbor, it's a tip
                if len(white_neighbors) == 1:
                    tips.append((x, y))
    
    return tips

def trace_branch(image, start_x, start_y, threshold=10):
    """Trace a branch from the start pixel until a junction or the threshold is reached."""
    x, y = start_x, start_y
    branch_pixels = [(x, y)]
    
    while True:
        # Get neighbors of the current pixel
        neighbors = get_neighbors(x, y, image.shape)
        white_neighbors = [(nx, ny) for nx, ny in neighbors if image[nx, ny] > 0]
        
        # Check the number of white neighbors
        if len(white_neighbors) == 1:
            # If exactly one white neighbor, we are at the end of the branch
            nx, ny = white_neighbors[0]
            branch_pixels.append((nx, ny))
            x, y = nx, ny
        elif len(white_neighbors) == 2:
            # If exactly two white neighbors, we are in the middle of the branch
            # Move to the next neighbor (continue tracing)
            nx, ny = [n for n in white_neighbors if (n[0], n[1]) != (x, y)][0]
            branch_pixels.append((nx, ny))
            x, y = nx, ny
        else:
            # If we have 3 or more white neighbors, we've reached a junction or the end
            break
        
        # Stop if the branch length exceeds the threshold
        if len(branch_pixels) > threshold:
            break
    
    return branch_pixels


def remove_short_branches(image, threshold=10):
    """Remove small offshoot branches from the image."""
    # Make a copy of the image to avoid modifying it during iteration
    cleaned_image = image.copy()
    
    # Find all the tips (pixels with exactly 1 white neighbor)
    tips = find_tips(image)
    print(tips)
    
    for tip in tips:
        start_x, start_y = tip
        branch_pixels = trace_branch(image, start_x, start_y, threshold)

        print(branch_pixels)
        
        # If the branch is shorter than the threshold, remove it
        if len(branch_pixels) < threshold:
            for (x, y) in branch_pixels:
                cleaned_image[x, y] = 0
    
    return cleaned_image


# Example usage
x = process_image_to_single_pixel_lines('transformed_image.png', 'test123.png')

y = remove_small_line_noise(x, 23)

skeleton_pruned = remove_short_branches(y)
cv2.imwrite('result.png', skeleton_pruned)