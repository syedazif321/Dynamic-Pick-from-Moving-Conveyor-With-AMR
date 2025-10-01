import cv2
# Import the module explicitly to ensure class access
from cv2 import aruco 
import numpy as np
import sys

def detect_aruco_in_image(image_path):
    """
    Loads an image from a path and detects ArUco markers.
    Prints the detected IDs and draws bounding boxes on the image.
    """
    print(f"Loading image from: {image_path}")
    
    # --- 1. Load the Image ---
    try:
        # Resolve the relative path to an absolute path for better reliability
        import os
        full_path = os.path.abspath(image_path)
        img = cv2.imread(full_path)
        
        if img is None:
            print(f"ERROR: Could not load image from path. Check if the file exists: {full_path}")
            return
    except Exception as e:
        print(f"An error occurred while reading the image: {e}")
        return

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # --- 2. Define the ArUco Dictionary and Parameters ---
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    # FIX for 'DetectorParameters_create' error
    parameters = aruco.DetectorParameters() 

    # --- 3. Detect Markers ---
    # FIX for 'detectMarkers' AttributeError: create a detector object
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    
    # Use the method from the detector object
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    
    # --- 4. Process Results and Visualize ---
    if ids is not None:
        print("\n✅ Markers Detected Successfully!")
        print(f"Total Markers Found: {len(ids)}")
        
        # Draw the detected markers and their IDs on the image
        aruco.drawDetectedMarkers(img, corners, ids, borderColor=(0, 255, 0)) # Green bounding box

        for i, marker_id in enumerate(ids):
            # Print the ID and the corner coordinates
            corner = corners[i][0]
            print(f"\nMarker ID: {marker_id[0]}")
            print(f"Corners (Top-Left, Top-Right, Bottom-Right, Bottom-Left):")
            print(f"  {corner}")

        # --- 5. Display the result ---
        cv2.imshow('ArUco Detection Test', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    else:
        print("\n❌ No ArUco markers detected in the image.")
        print("Please check: marker quality (blur/shadows), dictionary type (must be DICT_4X4_50), and image path.")
        
    print("-" * 30)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 test_aruco.py <path_to_image>")
        print("Example: python3 test_aruco.py /home/user/Pictures/aruco_screenshot.png")
    else:
        image_file_path = sys.argv[1]
        detect_aruco_in_image(image_file_path)