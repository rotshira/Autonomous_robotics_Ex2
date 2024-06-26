import sys

import cv2
import numpy as np
import csv


def eulerAnglesFromRotationMatrix(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.degrees(x), np.degrees(y), np.degrees(z)


def readBaselineCSV(file_path):
    data = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header
        for row in reader:
            frame_id = int(row[0])
            qr_id = int(row[1])
            qr_2d = eval(row[2])  # Convert string to list
            qr_3d = eval(row[3])  # Convert string to list
            data.append((frame_id, qr_id, qr_2d, qr_3d))
    return data


def computeMovementCommands(current_pose, target_pose, current_dist, target_dist):
    dist_diff = target_dist - current_dist
    yaw_diff = target_pose[1] - current_pose[0]
    pitch_diff = target_pose[2] - current_pose[1]
    roll_diff = target_pose[3] - current_pose[2]

    commands = []

    if abs(dist_diff) > 0.2:
        commands.append("backward" if dist_diff > 0 else "forward")

    if abs(roll_diff) > 0.25:
        commands.append("left" if roll_diff > 0 else "right")

    if abs(yaw_diff) > 5:
        commands.append("turn-left" if yaw_diff > 0 else "turn-right")

    if abs(pitch_diff) > 5:
        commands.append("down" if pitch_diff > 0 else "up")

    return commands if commands else "In Position"


def analyzeLiveVideo(baseline_data, target_frame_id):
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Webcam successfully connected.")

    target_baseline = next((entry for entry in baseline_data if entry[0] == target_frame_id), None)

    if not target_baseline:
        print(f"Error: No baseline data found for frame ID {target_frame_id}.")
        return

    target_qr_id = target_baseline[1]
    target_pose = target_baseline[3]  # [distance, yaw, pitch, roll]
    target_distance = target_pose[0]  # Assuming the first element in target_pose is distance

    in_position = False

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None:
            for i in range(len(ids)):
                id = ids[i][0]
                if id == target_qr_id:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, camera_matrix,
                                                                          dist_coeffs)
                    rvec = rvecs[0]
                    tvec = tvecs[0]  # Translation vector
                    current_distance = np.linalg.norm(tvec)
                    R, _ = cv2.Rodrigues(rvec)
                    current_pose = eulerAnglesFromRotationMatrix(R)  # [yaw, pitch, roll]

                    command = computeMovementCommands(current_pose, target_pose, current_distance, target_distance)

                    # Draw only the bounding box
                    cv2.aruco.drawDetectedMarkers(frame, corners)

                    yaw, pitch, roll = current_pose
                    cv2.putText(frame, f'Yaw: {yaw:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                                cv2.LINE_AA)
                    cv2.putText(frame, f'Pitch: {pitch:.2f}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                                cv2.LINE_AA)
                    cv2.putText(frame, f'Roll: {roll:.2f}', (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                                cv2.LINE_AA)
                    cv2.putText(frame, f'Distance: {current_distance:.2f}', (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2, cv2.LINE_AA)

                    if command:
                        cv2.putText(frame, f'Command: {command}', (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255),
                                    2, cv2.LINE_AA)
                        print("Movement Command:", command)
        else:
            cv2.putText(frame, 'No Id detected go backward', (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)

        cv2.imshow('Live Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()

    camera_matrix = np.array([[921.170702, 0.000000, 459.904354],
                              [0.000000, 919.018377, 351.238301],
                              [0.000000, 0.000000, 1.000000]])
    dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

    marker_length = 0.14  # ArUco marker size

    baseline_csv_file = 'target_frames.csv'  # Replace with your baseline CSV filename
    baseline_data = readBaselineCSV(baseline_csv_file)

    # target_frame_id = int(input("Enter the target frame ID from the baseline CSV: "))
    target_frame_id = int(sys.argv[1])
    analyzeLiveVideo(baseline_data, target_frame_id)
