import numpy as np
from shapely.geometry import Polygon
import cv2
import matplotlib.pyplot as plt

import rospy


# Define the clipping polygon
clipping_polygon_coords = np.array([
    [0, 0],    # Clipping polygon vertex 0
    [1280, 0],    # Clipping polygon vertex 1
    [1280, 720],    # Clipping polygon vertex 2
    [0, 720]     # Clipping polygon vertex 3
])

# Define the subject polygon to be clipped
subject_polygon_coords = np.array([
    [405, 217],    # Subject polygon vertex 0
    [397, 155],    # Subject polygon vertex 1
    [842-1280, -84+720],     # Subject polygon vertex 2
    [847-1280, -145+720],
])

# Convert numpy arrays to Shapely Polygon objects
clipping_polygon = Polygon(clipping_polygon_coords)
subject_polygon = Polygon(subject_polygon_coords)

# Perform Sutherland-Hodgman polygon clipping
clipped_polygon = subject_polygon.intersection(clipping_polygon)

# Print the resulting clipped polygon coordinates
print(np.array(clipped_polygon.exterior.coords))

image = np.zeros((720, 1280), dtype=np.uint8)

points = np.array(clipped_polygon.exterior.coords,np.int32)
print(points.shape)
points = points.reshape((-1, 1, 2))
print(points.shape)

if points.shape[0] != 0:
    cv2.fillPoly(image, [points], 255)

#cv2.imshow("Quadrilateral", image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

fig3 = plt.figure()
plt.imshow(image, cmap='gray')
plt.show()