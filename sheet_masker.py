import cv2
import numpy as np

# Load the image
image_path = "./test.png"  # Replace with your image file path
image = cv2.imread(image_path)

# Convert to HSV color space for better color segmentation
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the color range for the paper (adjust based on your image)
# Replace these values with appropriate HSV ranges for the paper's color
lower_color = np.array([0, 0, 110])  # Example: white paper
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

# Debug: Visualize the cleaned mask
cv2.imshow("Morphologically Cleaned Mask", cleaned_mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

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

