import cv2
import numpy as np

# Angle of rotation (CCW) between coordinate frames
THETA_RAD = np.radians(30)

class Camera:
    def __init__(self, camera_index=0):
        # Initialize the webcam
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise Exception("Error: Camera not accessible")

        # Define the ArUco dictionary and detector parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        
        # Define the HSV range for the orange ball
        self.lower_orange = np.array([5, 150, 150])
        self.upper_orange = np.array([30, 255, 255])

        # Real-world positions of the aruco markers in mm 
        self.real_world_points = np.array([
            [-135, -135],   # Bottom-left (ID 0)
            [135, -135],   # Top-left (ID 1)
            [135, 135],    # Top-right (ID 2)
            [-135, 135]   # Bottom-right (ID 3)
        ], dtype="float32")

    def compute_x_and_y(self):
        """
        Grab the most recent frame, detect the ball, and compute its coordinates
        relative to the robot's frame.
        """
        # Grab the latest frame
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to grab frame")

        # Convert to grayscale for ArUco marker detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # If all four corner markers are detected
        if ids is not None and len(ids) >= 4:
            # Map detected IDs to their corners
            marker_dict = {id[0]: corner for id, corner in zip(ids, corners)}

            # Ensure IDs 0 to 3 are detected
            if all(id in marker_dict for id in [0, 1, 2, 3]):
                # Arrange detected points in order of real-world points
                image_points = np.array([marker_dict[i][0][0] for i in [0, 1, 2, 3]], dtype="float32")

                # Compute the homography matrix
                homography_matrix, _ = cv2.findHomography(image_points, self.real_world_points)

                # Get the bounding box of the platform
                x_min, y_min = np.min(image_points, axis=0).astype(int)
                x_max, y_max = np.max(image_points, axis=0).astype(int)

                # Crop to the platform region
                cropped = frame[y_min:y_max, x_min:x_max]

                # Threshold to detect black areas (non-ball areas)
                mask = cv2.inRange(cropped, np.array([0, 0, 0]), np.array([80, 80, 80]))
                
                # Detect circles using Hough Transform
                blurred = cv2.GaussianBlur(mask, (9, 9), 2)
                circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                                           param1=50, param2=30, minRadius=10, maxRadius=100)

                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    for (x, y, r) in circles:
                        # Extract ROI for orange ball detection
                        hsv_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
                        roi = hsv_cropped[max(0, y-r):min(hsv_cropped.shape[0], y+r), 
                                          max(0, x-r):min(hsv_cropped.shape[1], x+r)]

                        # Check if the ROI contains orange
                        mask_orange = cv2.inRange(roi, self.lower_orange, self.upper_orange)
                        orange_pixels = cv2.countNonZero(mask_orange)
                        total_pixels = roi.shape[0] * roi.shape[1]

                        if orange_pixels / total_pixels > 0.3:  # Ball detected
                            ball_pixel = np.array([[x + x_min, y + y_min]], dtype="float32")
                            ball_mm = cv2.perspectiveTransform(np.array([ball_pixel]), homography_matrix)[0][0]

                            # Transform to robot frame
                            rotation_matrix = np.array([
                                [np.cos(THETA_RAD), -np.sin(THETA_RAD)],
                                [np.sin(THETA_RAD), np.cos(THETA_RAD)]
                            ])
                            ball_robot = np.dot(rotation_matrix, ball_mm)

                            return ball_robot[0], ball_robot[1]

        return None, None  # No ball detected

