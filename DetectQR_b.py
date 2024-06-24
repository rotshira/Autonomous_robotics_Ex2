import cv2
import numpy as np


def identify_markers_from_camera():
    """
    Identifies markers in the real-time video feed from the camera.
    """
    capture = cv2.VideoCapture(0)
    if not capture.isOpened():
        raise FileNotFoundError("Error - cannot access the camera")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

    while True:
        success, frame = capture.read()
        if not success:
            break

        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)
        if marker_ids is not None:
            for corners, marker_id in zip(marker_corners, marker_ids.flatten()):
                distance = 1 / np.linalg.norm(corners[0][0] - corners[0][1])
                yaw, pitch, roll = compute_orientation(corners[0], frame)
                display_command(frame, yaw, pitch)
                cv2.polylines(frame, [np.int32(corners)], True, (0, 255, 0), 2)

        cv2.imshow('Camera Screen', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()


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


def display_command(frame, yaw, pitch):
    """
    Determines and displays the movement command based on yaw and pitch.

    Args:
        frame (numpy.ndarray): Current video frame.
        yaw (float): Yaw angle.
        pitch (float): Pitch angle.
    """
    if yaw > 10:
        command = "move left"
    elif yaw < -10:
        command = "move right"
    elif pitch > 10:
        command = "move up"
    elif pitch < -10:
        command = "move down"
    else:
        command = "hold position"

    cv2.putText(frame, f"Command: {command}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    print(command)


def main():
    """
    Main function to detect markers in a live camera feed and display movement commands.
    """
    try:
        identify_markers_from_camera()
    except FileNotFoundError as error:
        print(error)
    except Exception as error:
        print("An unexpected error occurred:", error)


if __name__ == '__main__':
    main()
