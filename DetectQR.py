import argparse
import os

import numpy as np
import cv2
import csv


def identify_markers(video_path):
    """
    Identifies markers in the provided video file.

    Args:
        video_path (str): Path to the video file.

    Returns:
        list: List of detected markers in all frames.
    """
    capture = cv2.VideoCapture(video_path)
    if not capture.isOpened():
        raise FileNotFoundError("Error - problem with the file")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    markers_detected = []

    while capture.isOpened():
        success, frame = capture.read()
        if not success:
            break

        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)
        if marker_ids is not None:
            for corners, marker_id in zip(marker_corners, marker_ids.flatten()):
                distance = 1 / np.linalg.norm(corners[0][0] - corners[0][1])
                yaw, pitch, roll = compute_orientation(corners[0], frame)
                markers_detected.append(
                    (capture.get(cv2.CAP_PROP_POS_FRAMES), marker_id, corners[0], distance, yaw, pitch, roll))

    capture.release()
    return markers_detected


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


def save_markers_to_csv(file_path, markers_detected):
    """
    Exports detected marker information to a CSV file.

    Args:
        file_path (str): Path to the CSV file.
        markers_detected (list): List of detected markers in all frames.
    """
    with open(file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Frame", "Marker ID", "2D Coordinates", "Distance", "Yaw", "Pitch", "Roll"])
        for marker in markers_detected:
            frame_number, marker_id, marker_corners, distance, yaw, pitch, roll = marker
            coordinates = ','.join([f'({x},{y})' for x, y in marker_corners])
            writer.writerow([frame_number, marker_id, coordinates, distance, yaw, pitch, roll])


def annotate_and_save_video(input_video, output_video, markers_detected):
    """
    Annotates and saves the video with detected markers.

    Args:
        input_video (str): Path to the input video file.
        output_video (str): Path to the output video file.
        markers_detected (list): List of detected markers in all frames.
    """
    capture = cv2.VideoCapture(input_video)
    if not capture.isOpened():
        raise FileNotFoundError("Error - problem with the file")

    fps = capture.get(cv2.CAP_PROP_FPS)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    video_writer = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

    current_frame = 0
    for marker in markers_detected:
        frame_number, marker_id, marker_corners, _, _, _, _ = marker
        while current_frame < frame_number:
            success, frame = capture.read()
            if not success:
                capture.release()
                video_writer.release()
                return
            current_frame += 1
            video_writer.write(frame)

        cv2.polylines(frame, [np.int32(marker_corners)], True, (0, 255, 0), 2)
        print(f"Markers annotated in frame {current_frame}")
        video_writer.write(frame)

    capture.release()
    video_writer.release()


def main():
    """
    Main function to detect markers in a video, export results to CSV, and annotate the video.
    """
    parser = argparse.ArgumentParser(description='Detect markers in a video file.')
    parser.add_argument('video_file_path', type=str, help='Path to the input video file')

    args = parser.parse_args()
    video_file_path = args.video_file_path
    base_name = os.path.splitext(os.path.basename(video_file_path))[0]
    csv_file_path = f'{base_name}_detection_frames.csv'
    annotated_video_path = f'{base_name}_with_markers.mp4'

    try:
        detected_markers_list = identify_markers(video_file_path)

        if detected_markers_list:
            save_markers_to_csv(csv_file_path, detected_markers_list)
            annotate_and_save_video(video_file_path, annotated_video_path, detected_markers_list)
            print("Detection results saved to", csv_file_path)
            print("Annotated video saved to", annotated_video_path)
        else:
            print("No markers detected in the given video")

    except FileNotFoundError as error:
        print(error)
    except Exception as error:
        print("An unexpected error occurred:", error)


if __name__ == '__main__':
    main()
