# ARIC Camera Calibration Routine | Prophesee EVK4 - HD

<!-- &nbsp; -->
<img src=".readme/1010.jpg" />
<!-- &nbsp; -->

**NOTE**: This package uses an old, unoptimized version of `ros_robot_pkg`, but it's working!

## Requirements

This pipeline was tested using:

- Ubuntu 20.04 64-bit
- ROS Noetic
- Python 3.8.10
- Prophesee EVK4 - HD
- DV ROS
- Metavision SDK (OpenEB) 3.0.2
- Metavision ROS Driver (unofficial)
- OpenCV contrib = 4.8.0.76

## Installation
This routine requires Python >= 3.8.10 and we strongly recommend the use of virtual environments (pyenv or venv).

Also make sure you have ROS sourced properly such that ROS_VERSION is set:

```bash
source /opt/ros/noetic/setup.bash
```

Clone this repository in your catkin workspace:

```bash
cd catkin_ws/src
git clone https://github.com/AdvancedResearchInnovationCenter/aric-camera-calibration.git
cd aric-camera-calibration
git checkout prophesee-camera-calibration
pip install -r requirements.txt
```
And install [DV_ROS](https://gitlab.com/inivation/dv/dv-ros) in a separate workspace.

We calibrate both intrinsic and extrinsic parameters of the Prophesee EVK4 - HD by accumulating events over time and convert them to grayscale images using `dv_ros_accumulator`. 

To install the Metavision ROS Driver (unofficial but faster), install `python3-wstool` and follow the instructions below:

```
mkdir -p ~/metavision_ros_driver_ws/src
cd ~/metavision_ros_driver_ws
git clone git@github.com:berndpfrommer/metavision_ros_driver src/metavision_ros_driver
git checkout e612eb906b9371954e428b82117bba39b26ca749
wstool init src src/metavision_ros_driver/metavision_ros_driver.rosinstall
```

Before compiling the ROS driver we need to swap 'event_array_msgs' with our customized version included in `aric-camera-calibration` folder:

```
cd ..
rm -rf event_array_msgs
cp -r ~/catkin_ws/src/aric-camera-calibration/event_array_msgs ~/metavision_ros_driver_ws/src
```

Now configure and build:

```
cd ..
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

Also compile catkin workspace:

```
cd ~/catkin_ws
catkin_make
```

Now make sure `aric-camera-calibration`,`dv-ros` and `metavision_ros_driver` are properly sourced (tip: include it in your .bashrc):

```
source ~/catkin_ws/devel/setup.bash
source ~/dv_ros/devel/setup.bash
source ~/dv_ros/devel/setup.bash
```

Connect the Prophesee camera. Check that you receive the events correctly:

```
metavision_viewer
```

Now quit `metavision_viewer` and run `run_pipeline.sh`. Alternatively, run each terminal independently (make sure each time that the workspaces are sourced properly).

```
cd src/aric-camera-calibration
chmod +x run run_pipeline.sh
./run_pipeline.sh
```

 `rqt_image_view` allows you to select the rostopic with grayscale stream `/accumulator/image`, and `rqt_reconfigure` lets you play with the accumulation and camera parameters. More info can be found [here](https://docs.prophesee.ai/stable/hw/manuals/biases.html).

If you arrived here successfully, congrats! We can now procede with the actual calibration:


1. Generate `calibration_config.json` as per your setup. Follow `event_camera_calibration_config.json` for guidance, replace x, y, z, and the rotation matrix as per your setup, otherwise you'll get an error. (Currently, **ChAruCo board** is the only supported calibration target)
2. Generate a `tool.urdf.xacro` as per your tool. Follow `sample_tool.urdf.xacro` for guidance
3. Run:
  ```
  roslaunch aric-camera-calibration bringup_calibration.launch
  ```
  tip: if you find the focus to be very narrow in height, reduce the amount of incoming light by reducing the aperture of the camera!
4. Calibration data will be saved in a new directory:
   ```
   ros_robot
     ├── CMakeLists.txt
     ├── README.md
     ├── calibration_data  <-- NEW DIRECTORY CREATED FOR YOUR CALIBRATION DATA
     ├── launch
     ├── package.xml
     ├── scripts
     ├── srv
     └── xacros
   ```

## example `calibration_config.json `

```json
{
    "robot": {
        "name": "UR10",
        "ip": "192.168.50.110"
    },
    "calibration_target": {
        "type": "charuco",
        "size": [11,8],
        "checker_length": 0.022,
        "marker_length": 0.016,
        "legacy_pattern": true,
        "aruco_dict": "DICT_4X4_250",
        "blur":[11,2],
        "target2base": [[-1, 0,  0, 0.055],
                        [ 0, 1,  0, -0.53],
                        [ 0, 0, -1,     0],
                        [ 0, 0,  0,     1]]
    },
    "calibration_data": {
        "project_name": "tactile_ov7521_3",
        "image_topic": "/ardu_cam/image_raw",
        "data_collection_setup": [0.4, 5, 2, 0.10, 0.15],
        "output_file_name": "tactile_ov7521"
    }
}
```

### 1. `robot`

Info about the robot used

|      Key | Description                |             Value             |  Type  |
| -------: | -------------------------- | :----------------------------: | :----: |
| `name` | Specifies the robot in use | "UR10", "ABB", or "Mitsubishi" | string |
|   `ip` | Robot's IP address         |   _e.g._ "192.168.50.110"   | string |

### 2. `calibration_target`

Info about the calibration target

|                Key | Description                                                                                                                 |                                                                                                              Value                                                                                                              |         Type         |                                                                               Note                                                                               |
| -----------------: | --------------------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :-------------------: | :---------------------------------------------------------------------------------------------------------------------------------------------------------------: |
|           `type` | Calibration target type                                                                                                     |                                                                                                 "charuco", "aruco", or "checker"                                                                                                 |        string        |                                                                Currently, only "charuco" will work                                                                |
|           `size` | Calibration target size                                                                                                     |                                                                                                 [rows x cols] (_e.g._ [11,8])                                                                                                 |      [int, int]      |                                                               num of squares,_NOT_ inner corners                                                               |
| `checker_length` | Charuco/chess board black square length (m)                                                                                 |                                                                                                          _e.g._ 0.022                                                                                                          |         float         |                                                  It's better to measure it after printing the calibration target                                                  |
|  `marker_length` | Charuco/aruco marker length (m)                                                                                             |                                                                                                          _e.g._ 0.016                                                                                                          |         float         |                                                  It's better to measure it after printing the calibration target                                                  |
| `legacy_pattern` | This is related to Charuco targets.`<br>`It specifies whether you are using the old or the new  pattern for Charuco board |                                                                                                            True/False                                                                                                            |         bool         |                                Check this[issue](https://github.com/opencv/opencv/issues/23873#issuecomment-1620504453) for more info                                |
|     `aruco_dict` | Which aruco dictionary is in use                                                                                            |                                                                                                    _e.g._ `DICT_4X4_250`                                                                                                    |        string        | Use the same naming pattern as in `cv2.aruco` library [here](https://docs.opencv.org/4.8.0/de/d67/group__objdetect__aruco.html#ga4e13135a118f497c6172311d601ce00d) |
|           `blur` | Gaussian blur parameters used to smoothen the captured images                                                               |                                                                                       [kernel_size, standard_deviation] (_e.g._  [11,2])                                                                                       |     [int, float]     |                      [cv2.GaussianBlur()](https://docs.opencv.org/4.8.0/d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1)                      |
|    `target2base` | The transformation matrix from the calibration target to the robot base                                                     | $`{}^{base}T_{target} = \begin{bmatrix}  R_{3×3} & T_{3×1} \\[0.5em] 0_{1×3} & 1 \end{bmatrix}`$ `<br>` _e.g._ `<br>` [[-1, 0,  0, 0.055],`<br>`[ 0, 1,  0, -0.53],`<br>` [ 0, 0, -1, 0],`<br>` [ 0, 0,  0, 1]] | float (list of lists) |                            For convenience, we set the calibration target`<br>` orientation to be the same as the camera orientation                            |

### 3. `calibration_data`

Info about the output calibration data

| Key                       | Description                       | Value                                                                                                      | Type                            | Note                                                                                              |
| ------------------------- | --------------------------------- | ---------------------------------------------------------------------------------------------------------- | ------------------------------- | ------------------------------------------------------------------------------------------------- |
| `project_name`          | Your project name                 | _e.g._ "D435_calibration"                                                                                | string                          | A new directory named after the project will be`<br>` created to store all the calibration data |
| `image_topic`           | Inbound image stream              | _e.g._ "/image/raw"                                                                                      | string                          | The input `image_topic` will be subscribed to                                                   |
| `data_collection_setup` | Random pose generation parameters | [$`\phi`$, n_cycle, n_pose_per_cycle, min_radius, max_radius] `<br>` _e.g._ [0.4, 5, 20, 0.10, 0.15] | [float, int, int, float, float] |                                                                                                   |
| `output_file_name`      | Pose-image pairs data JSON file   | _e.g._ "D435_calibration_2nd.json"                                                                       | string                          |                                                                                                   |

## Sample Output

```txt
+------------------------------+
| INTRINSIC CAMERA CALIBRATION |
+------------------------------+
* Reprojection Error: 0.3585812344507631
* Camera Matrix
  [438.59222618   0.         323.89686632]
  [  0.         439.81991094 240.11146683]
  [0. 0. 1.]
* Distortion Coefficients
  [0.11232303654662454, -0.47277989637474027, -6.726553087628563e-05, 0.0005557680260483787, 0.5754486589853096]

+------------------------------+
| EXTRINSIC CAMERA CALIBRATION |
+------------------------------+
* tcp_T_cam
  [ 0.99841971 -0.00733935  0.0557155  -0.00005566]
  [0.00560671 0.99949772 0.03119081 0.00015634]
  [-0.05591644 -0.03082914  0.99795938  0.13995561]
  [0. 0. 0. 1.]

* Translation
  [-0.00005566  0.00015634  0.13995561]

* Euler Angles (ZYX)
  (rad) X: -0.03088235865331096	Y: 0.055945617151973215	Z: 0.005615524215201508
  (deg) X: -1.7694288122440347	Y: 3.205447745062774	Z: 0.32174583728456024
```
