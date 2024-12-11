import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

# Load the map
map_path = 'maze1_smallest(1)/run_3/map.pgm'  # Replace with your map file path

# Check if the file exists
if not os.path.exists(map_path):
    print(f"File not found: {map_path}")
    exit()

# Try to read the image
map_image = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)

# If the map can't be read, show an error message
if map_image is None:
    print(f"Error: Unable to read the image file at {map_path}")
    exit()

# Verify the min and max values in the map
max_val = np.max(map_image)
min_val = np.min(map_image)
print(f"Max value: {max_val}")
print(f"Min value: {min_val}")

# Plot the histogram to help identify free space value
plt.hist(map_image.ravel(), bins=256, range=(0, 256))
plt.title("Pixel Value Distribution")
plt.xlabel("Pixel Value")
plt.ylabel("Frequency")

# Save the histogram plot to the source directory
histogram_path = os.path.join(os.path.dirname(map_path), 'histogram.png')
plt.savefig(histogram_path)
print(f"Histogram saved to: {histogram_path}")

# Show the histogram plot
plt.show()

# Count the number of free cells (assuming free space value is 254)
free_space_value = 254
open_cells = np.sum(map_image == free_space_value)
print(f"Number of open cells: {open_cells}")

# Get the resolution from the map.yaml (replace with actual resolution)
resolution = 0.05  # change according to your map.yaml
cell_area = resolution ** 2

# Calculate the covered area
covered_area = open_cells * cell_area
print(f"Covered Area: {covered_area} m²")

# Visualize the map with the identified free space
plt.imshow(map_image, cmap='gray')
plt.colorbar()
plt.title("Map Visualization")

# Save the map visualization to the source directory
map_visualization_path = os.path.join(os.path.dirname(map_path), 'map_visualization.png')
plt.savefig(map_visualization_path)
print(f"Map visualization saved to: {map_visualization_path}")

# Show the map visualization plot
plt.show()
