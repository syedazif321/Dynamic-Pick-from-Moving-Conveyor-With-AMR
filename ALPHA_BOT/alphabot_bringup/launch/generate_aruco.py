import cv2
from cv2 import aruco
import numpy as np

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker_id = 0
marker_size = 1000

marker_img = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# Add a white border around (quiet zone, 1 "cell" wide)
border_size = marker_size // 7  # ~1 module width
marker_with_border = cv2.copyMakeBorder(
    marker_img,
    border_size, border_size, border_size, border_size,
    cv2.BORDER_CONSTANT,
    value=255
)

cv2.imwrite("aruco_with_border.png", marker_with_border)
print("✅ Saved aruco_with_border.png with border")
