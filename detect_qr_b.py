import sys
import cv2
import numpy as np
import csv

# Function to calculate Euler angles from a rotation matrix
def eulerAnglesFromRotationMatrix(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])  # sy calculation for singular case check
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])  # Calculate x (yaw)
        y = np.arctan2(-R[2, 0], sy)  # Calculate y (pitch)
        z = np.arctan2(R[1, 0], R[0, 0])  # Calculate z (roll)
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])  # Calculate x (yaw)
        y = np.arctan2(-R[2, 0], sy)  # Calculate y (pitch)
        z = 0

    return np.degrees(x), np.degrees(y), np.degrees(z)  # Convert angles to degrees and return


# Function to read baseline data from a CSV file
def readBaselineCSV(file_path):
    data = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header
        for row in reader:
            frame_id = int(row[0])
            qr_id = int(row[1])
            qr_2d = eval(row[2])  # Convert string representation to list
            qr_3d = eval(row[3])  # Convert string representation to list
            data.append((frame_id, qr_id, qr_2d, qr_3d))
    return data


# Function to compute movement commands based on current and target pose
def computeMovementCommands(current_pose, target_pose, current_dist, target_dist):
    # Calculate differences between current and target positions and orientations
    dist_diff = target_dist - current_dist
    yaw_diff = target_pose[1] - current_pose[0]
    pitch_diff = target_pose[2] - current_pose[1]
    roll_diff = target_pose[3] - current_pose[2]

    commands = []

    # Determine movement commands based on the differences
    if abs(dist_diff) > 0.2:
        commands.append("backward" if dist_diff > 0 else "forward")

    if abs(roll_diff) > 0.25:
        commands.append("left" if roll_diff > 0 else "right")

    if abs(yaw_diff) > 5:
        commands.append("turn-left" if yaw_diff > 0 else "turn-right")

    if abs(pitch_diff) > 5:
        commands.append("down" if pitch_diff > 0 else "up")

    return commands if commands else "In Position"


# Function to analyze live video stream and detect markers
def analyzeLiveVideo(baseline_data, target_frame_id):
    # Open the webcam
    cap = cv2.VideoCapture(0)

    # Check if the webcam is opened successfully
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Webcam successfully connected.")

    # Find baseline data for the target frame ID
    target_baseline = next((entry for entry in baseline_data if entry[0] == target_frame_id), None)

    if not target_baseline:
        print(f"Error: No baseline data found for frame ID {target_frame_id}.")
        return

    # Extract relevant information from baseline data
    target_qr_id = target_baseline[1]
    target_pose = target_baseline[3]  # [distance, yaw, pitch, roll]
    target_distance = target_pose[0]  # Assuming the first element in target_pose is distance

    while cap.isOpened():
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            break

        # Detect markers in the current frame
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None:
            for i in range(len(ids)):
                id = ids[i][0]
                if id == target_qr_id:
                    # Estimate pose of the marker
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, camera_matrix,
                                                                          dist_coeffs)
                    rvec = rvecs[0]
                    tvec = tvecs[0]  # Translation vector
                    current_distance = np.linalg.norm(tvec)
                    R, _ = cv2.Rodrigues(rvec)
                    current_pose = eulerAnglesFromRotationMatrix(R)  # [yaw, pitch, roll]

                    # Compute movement commands based on current and target poses
                    command = computeMovementCommands(current_pose, target_pose, current_distance, target_distance)

                    # Draw detected markers and annotate frame with pose information
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    yaw, pitch, roll = current_pose

                    # Adjusted position for text annotations
                    text_position = (10, 30)
                    text_spacing = 40

                    # Display pose information at the top of the frame
                    cv2.putText(frame, f'Yaw: {yaw:.2f}', text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                                cv2.LINE_AA)
                    text_position = (text_position[0], text_position[1] + text_spacing)
                    cv2.putText(frame, f'Pitch: {pitch:.2f}', text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                                cv2.LINE_AA)
                    text_position = (text_position[0], text_position[1] + text_spacing)
                    cv2.putText(frame, f'Roll: {roll:.2f}', text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                                cv2.LINE_AA)
                    text_position = (text_position[0], text_position[1] + text_spacing)
                    cv2.putText(frame, f'Distance: {current_distance:.2f}', text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 255, 0), 2, cv2.LINE_AA)

                    # Display movement command if available
                    if command:
                        text_position = (text_position[0], text_position[1] + text_spacing)
                        cv2.putText(frame, f'Command: {command}', text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255),
                                    2, cv2.LINE_AA)
                        print("Movement Command:", command)
        else:
            cv2.putText(frame, 'No Id detected, go backward', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
                        cv2.LINE_AA)

        # Display the annotated frame
        cv2.imshow('Live Video', frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Initialize ArUco dictionary and detector parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()

    # Define camera matrix (intrinsic parameters)
    camera_matrix = np.array([[921.170702, 0.000000, 459.904354],
                              [0.000000, 919.018377, 351.238301],
                              [0.000000, 0.000000, 1.000000]])
    dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

    marker_length = 0.14  # ArUco marker size

    baseline_csv_file = 'target_frames.csv'  # Replace with your baseline CSV filename
    baseline_data = readBaselineCSV(baseline_csv_file)

    # Read target frame ID from command line arguments
    target_frame_id = int(sys.argv[1])
    analyzeLiveVideo(baseline_data, target_frame_id)
