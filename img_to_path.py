import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize

# Load the image
image_path = 'transformed_image.png'
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
image = ~image

(thresh, im_bw) = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

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



# Find contours in the edge-detected image
contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

print(contours)

# Convert contours to paths (lists of coordinates)
paths = [contour[:, 0, :].tolist() for contour in contours]

# Visualize the edges and contours
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
plt.title("Edge Detection")
plt.imshow(binary, cmap='gray')

plt.subplot(1, 2, 2)
plt.title("Contours")
contour_image = np.zeros_like(skeleton)
#print(contours)
#cv2.drawContours(contour_image, contours, -1, (255), 1)  # Draw contours for visualization



print("Number of paths:", len(paths))
print("First path coordinates:", paths[0][:10])  # Display the first 10 points of the first path

eps = 0.01
newpaths = []
for path in contours:
    path = np.array(path)
    peri = cv2.arcLength(path, True)
    approx = cv2.approxPolyDP(path, eps*peri, False)
    #print([i[0] for i in approx])
    newpaths.append(approx)
print(newpaths)
cv2.drawContours(contour_image, newpaths, -1, (255), 1)

plt.imshow(contour_image, cmap='gray')
plt.show()

# Print example paths
