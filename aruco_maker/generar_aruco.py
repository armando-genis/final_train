import cv2 as cv  # Import the OpenCV library with alias cv
import cv2.aruco as aruco  # Import the ArUco submodule from OpenCV
import numpy as np

# Load the predefined dictionary
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
# This line retrieves a dictionary containing ArUco marker designs.
# We're using DICT_6X6_250 which means a 6x6 bit grid dictionary with 250 possible IDs.

# Generate the marker image
markerImage = np.zeros((200, 200), dtype=np.uint8)
# This line creates a black (all zeros) image with a size of 200x200 pixels.
# The dtype=np.uint8 specifies that each pixel value will be an unsigned 8-bit integer (0 to 255).

markerImage = aruco.generateImageMarker(dictionary, 4, 200)
# This line generates the ArUco marker on the markerImage.
# - dictionary: The predefined dictionary retrieved earlier.
# - 33: This is the ID of the marker you want to generate. You can change this value to create markers with different IDs.
# - 200: This defines the size (side length) of the marker in pixels within the image.

# Save the marker image
cv.imwrite("marker_4.png", markerImage)
# This line saves the generated marker image (markerImage) as "marker33.png".