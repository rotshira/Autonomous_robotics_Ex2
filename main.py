import cv2
import numpy as np
import csv

def detect_markers(video_path):
    capture = cv2.VideoCapture(video_path)
    if not capture.isOpened():
        print("Error: Couldn't open video file")
        return None, None

    aruco_dicts = [cv2.aruco.DICT_4X4_100]

    all_detected_markers = []
    marker_count_per_frame = []
    frame_index = 0

    while capture.isOpened():
        success, frame = capture.read()
        if not success:
            break

        frame_markers_info = []
        detected_ids_set = set()

        for aruco_dict_type in aruco_dicts:
            aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
            corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)

            if ids is not None:
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    if marker_id not in detected_ids_set:
                        marker_corners = corners[i][0]
                        distance = estimate_distance(marker_corners)
                        yaw, pitch, roll = compute_orientation(marker_corners, frame)
                        frame_markers_info.append((frame_index, marker_id, marker_corners, distance, yaw, pitch, roll))

                        detected_ids_set.add(marker_id)

        marker_count = len(frame_markers_info)
        print(f"Total markers detected in frame {frame_index}: {marker_count}")
        all_detected_markers.append(frame_markers_info)
        marker_count_per_frame.append(marker_count)
        frame_index += 1

    capture.release()

    return all_detected_markers, marker_count_per_frame

def estimate_distance(corners):
    side_length = np.linalg.norm(corners[0] - corners[1])
    distance = 1 / side_length  # Simplified estimation; calibrate with real measurements
    return distance

def compute_orientation(corners, frame):
    vector1 = corners[1] - corners[0]
    vector2 = corners[3] - corners[0]

    roll = np.degrees(np.arctan2(vector1[1], vector1[0]))

    center = np.mean(corners, axis=0)
    delta_x = center[0] - frame.shape[1] / 2
    delta_y = center[1] - frame.shape[0] / 2

    yaw = np.degrees(np.arctan2(delta_x, frame.shape[1]))
    pitch = -np.degrees(np.arctan2(delta_y, frame.shape[0]))

    return yaw, pitch, roll

def write_csv(output_path, detected_markers):
    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Frame", "Marker ID", "2D Coordinates", "Distance", "Yaw", "Pitch", "Roll"])

        for markers_in_frame in detected_markers:
            for frame_number, marker_id, marker_corners, distance, yaw, pitch, roll in markers_in_frame:
                frame_coordinates = ','.join([f'({x},{y})' for x, y in marker_corners])
                writer.writerow([frame_number, marker_id, frame_coordinates, distance, yaw, pitch, roll])

def draw_annotations(video_path, output_video_path, detected_markers):
    capture = cv2.VideoCapture(video_path)
    if not capture.isOpened():
        print("Error: Couldn't open video file")
        return

    fps = capture.get(cv2.CAP_PROP_FPS)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    frame_index = 0
    for markers_in_frame in detected_markers:
        success, frame = capture.read()
        if not success:
            break

        for frame_number, marker_id, marker_corners, distance, yaw, pitch, roll in markers_in_frame:
            rect_color = (0, 255, 0)
            cv2.polylines(frame, [np.int32(marker_corners)], True, rect_color, 2)

            org = (int(marker_corners[0][0]), int(marker_corners[0][1]) - 20)
            cv2.putText(frame, str(marker_id), org, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        print(f"{len(markers_in_frame)} markers annotated in frame {frame_index}")

        video_writer.write(frame)
        frame_index += 1

    capture.release()
    video_writer.release()

def main():
    video_path = 'video/challengeB.mp4'
    csv_output_path = 'detection_results.csv'
    video_output_path = 'annotated_video.mp4'

    detected_markers, _ = detect_markers(video_path)

    if detected_markers:
        write_csv(csv_output_path, detected_markers)
        draw_annotations(video_path, video_output_path, detected_markers)

        print("Detection results saved to", csv_output_path)
        print("Annotated video saved to", video_output_path)
    else:
        print("No markers detected in the video")

if __name__ == "__main__":
    main()
