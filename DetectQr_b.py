import cv2
import numpy as np


def compute_orientation(corners, frame):
    """
    Computes the orientation (yaw, pitch, roll) of the marker.

    Args:
        corners (numpy.ndarray): Array of marker corner coordinates.
        frame (numpy.ndarray): Current video frame.

    Returns:
        tuple: (yaw, pitch, roll) of the marker.
    """
    vector = corners[1] - corners[0]
    roll = np.degrees(np.arctan2(vector[1], vector[0]))

    center = np.mean(corners, axis=0)
    delta_x = center[0] - frame.shape[1] / 2
    delta_y = center[1] - frame.shape[0] / 2

    yaw = np.degrees(np.arctan2(delta_x, frame.shape[1]))
    pitch = -np.degrees(np.arctan2(delta_y, frame.shape[0]))

    return yaw, pitch, roll


def detect_qr_centers(frame):
    """
    Detects QR code centers in a given frame.

    Args:
        frame (numpy.ndarray): The frame to detect QR codes in.

    Returns:
        list: List of detected QR code centers and their IDs.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    aruco_params = cv2.aruco.DetectorParameters()
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
    centers = []
    ids = []

    if marker_ids is not None:
        for corners, marker_id in zip(marker_corners, marker_ids.flatten()):
            center_x = int(np.mean([p[0] for p in corners[0]]))
            center_y = int(np.mean([p[1] for p in corners[0]]))
            centers.append((center_x, center_y))
            ids.append(marker_id)
    return centers, ids


# Capture the target frame from a file or a static image
target_frame_path = 'target_frame.jpg'
target_frame = cv2.imread(target_frame_path)
if target_frame is None:
    print("Error: Could not read target frame.")
    exit()

target_centers, target_ids = detect_qr_centers(target_frame)

# Draw detected QR centers on the target frame for visualization
for center in target_centers:
    cv2.circle(target_frame, center, 5, (0, 255, 0), -1)
cv2.imshow('Target Frame', target_frame)
cv2.waitKey(0)
cv2.destroyAllWindows()


def generate_movement_commands(target_centers, live_centers):
    if not target_centers or not live_centers:
        return ["No QR detected"]

    commands = []

    for live_center in live_centers:
        closest_target = min(target_centers, key=lambda x: np.linalg.norm(np.array(live_center) - np.array(x)))

        dx = closest_target[0] - live_center[0]
        dy = closest_target[1] - live_center[1]

        if abs(dx) > 20:  # Threshold for movement
            if dx > 0:
                commands.append("right")
            else:
                commands.append("left")

        if abs(dy) > 20:  # Threshold for movement
            if dy > 0:
                commands.append("down")
            else:
                commands.append("up")

    if not commands:
        commands.append("aligned")

    return commands

try:
        # Initialize video capture from the PC camera
        cap = cv2.VideoCapture(0)  # 0 refers to the default camera
        if not cap.isOpened():
            print("Error: Could not open video capture.")
            exit()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame from video capture.")
                break

            live_centers, live_ids = detect_qr_centers(frame)

            for center in live_centers:
                cv2.circle(frame, center, 5, (0, 255, 0), -1)

            commands = generate_movement_commands(target_centers, live_centers)
            print("Commands:", commands)

            # Display commands on the frame
            command_text = ' | '.join(commands)
            cv2.putText(frame, command_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow('Live Video', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
finally:
    # Release the camera and destroy all windows
    cap.release()
    cv2.destroyAllWindows()
    print("Camera shut down and windows closed.")
# cap.release()
# cv2.destroyAllWindows()
