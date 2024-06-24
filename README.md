# TelloAI V.0.1 - Indoor Autonomous Drone Competition
## Autonomous Robotics Ex2 - Part A

This project is part A (out of two) of a course "Autonomous Robotics".
On this part we focus on the main image and video processing challenge: detecting QR codes within a video. The task involves processing each frame of the video to identify any QR codes and recording specific parameters for each detected QR code.
Using the [ArUco markers](https://github.com/tentone/aruco) library we could achieve that goal providing us an easier "built-in" solution for such problem.

## Requirements

- `numpy`
- `opencv-python-headless`
- `opencv-contrib-python-headless`

## Installation

1. Clone the repository to your local machine:

    ```sh
    git clone <https://github.com/rotshira/Autonomous_robotics_Ex2.git>
    cd <Autonomous_robotics_Ex2>
    ```

2. Install the required packages:

    ```sh
    pip install -r requirements.txt
    ```

## Usage

1. Ensure you have a video with markers in it.

2. Run the program:

    ```sh
    python DetectQR.py path/to/video.mp4

    ```

    Replace `<path/to/video>` with the path to your map image file (e.g., `src/video.mp4`).

## Output
* A CSV file containing the detection results will be generated. The file will be named based on the input video file name, with _detection_frames.csv appended. 
* An annotated video with detected markers will be generated. The file will be named based on the input video file name, with _with_markers.mp4 appended.

# Functions
**identify_markers(video_path)**
Identifies markers in the provided video file.

* Args:
  * `video_path` (str): Path to the video file.
* Returns:
  * list: List of detected markers in all frames.

**compute_orientation(corners, frame)**
Computes the orientation (yaw, pitch, roll) of the marker.

* Args:
  * `corners` (numpy.ndarray): Array of marker corner coordinates.
  * `frame` (numpy.ndarray): Current video frame.
* Returns:
  * tuple: (yaw, pitch, roll) of the marker.
  
**save_markers_to_csv(file_path, markers_detected)**
  Exports detected marker information to a CSV file.

* Args:
   * `file_path` (str): Path to the CSV file.
   * `markers_detected` (list): List of detected markers in all frames.

**annotate_and_save_video(input_video, output_video, markers_detected)**
     Annotates and saves the video with detected markers.

* Args:
  * `input_video` (str): Path to the input video file.
  * `output_video` (str): Path to the output video file.
  * `markers_detected` (list): List of detected markers in all frames.

**main():**
  Main function to detect markers in a video, export results to CSV, and annotate the video.

## Example
````
Replace `script_name.py` with the actual name of your Python script in the `README.md` file.

These files provide clear instructions on installing dependencies and running the script, making it easy for others to understand and use your project.
 challengeB.mp4, detect markers, save the detection results to challengeB_detection_frames.csv, and save the annotated video to challengeB_with_markers.mp4.
````

## Footage

### **Detected/Undetected from a video:**
<img width="1292" alt="image" src="https://imgur.com/ghtwNzz.png">

### Part of the CSV file 'results.csv':
| Frame | Marker ID | 2D Coordinates | Distance | Yaw | Pitch | Roll |
|-------|-----------|----------------|----------|-----|-------|------|
| 185.0 | 8         | "(371.0,4.0),(429.0,3.0),(431.0,64.0),(373.0,65.0)" | 0.01723881690412039 | -4.704365105774587 | 24.359961027687056 | -0.98776037 |
| 186.0 | 8         | "(371.0,11.0),(429.0,10.0),(431.0,71.0),(373.0,72.0)" | 0.01723881690412039 | -4.704365105774587 | 23.896003442838886 | -0.98776037 |
| 187.0 | 8         | "(372.0,28.0),(430.0,27.0),(432.0,87.0),(374.0,88.0)" | 0.01723881690412039 | -4.645078425891606 | 22.789169885991623 | -0.98776037 |
| 188.0 | 8         | "(372.0,35.0),(430.0,34.0),(432.0,93.0),(374.0,95.0)" | 0.01723881690412039 | -4.645078425891606 | 22.331088171045955 | -0.98776037 |
| 189.0 | 8         | "(372.0,40.0),(429.0,38.0),(432.0,98.0),(374.0,99.0)" | 0.01753307030785004 | -4.659901035460684 | 22.02401599737297 | -0.98776037 |
