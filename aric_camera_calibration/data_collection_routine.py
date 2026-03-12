#!/usr/bin/python3
import json
import math
import os
import pathlib
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import StaticTransformBroadcaster

from aric_camera_calibration.utils import ARUCO_DICT, ORN, RST

CONFIG = os.path.join(os.path.dirname(__file__), "../config/calibration_config.json")


class CameraCalibrationDataCollection(Node):

    def __init__(self, config_file: str = CONFIG):
        super().__init__('camera_calibration_data_collection')
        self._load_config(config_file)
        self._setup_camera()
        self._setup_target()

        self.data_counter          = 0
        self.calibration_data_list = {}

        if self.use_existing_data:
            return

        self.data_collected = False

        if self.robot_name == 'Doosan':
            from aric_camera_calibration.doosan_ros2 import DoosanRobot
            self.robot = DoosanRobot()
        # elif self.robot_name == 'UR':
        #     from aric_camera_calibration.ur_ros2 import URRobot
        #     self.robot = URRobot()
        # elif self.robot_name == 'ABB':
        #     from aric_camera_calibration.abb_ros2 import ABBRobot
        #     self.robot = ABBRobot()
        else:
            raise RuntimeError(f"Unknown robot '{self.robot_name}'. "
                               f"Add your robot wrapper and register it here.")

    def _load_config(self, config_file: str):
        with open(config_file) as f:
            cfg = json.load(f)

        self.calibration_config = cfg
        self.robot_name = cfg['robot']['name']

        tgt = cfg['calibration_target']
        self.target_type      = tgt['type']
        self.target_size      = tuple(tgt['size'])
        self.target_aruco_dict = ARUCO_DICT[tgt['aruco_dict']]
        self.checker_length   = tgt['checker_length']
        self.marker_length    = tgt['marker_length']
        self.legacy_pattern   = tgt['legacy_pattern']
        self.blur             = tgt['blur']

        cam = cfg['camera']
        self.ros_image_topic     = cam['image_topic']
        self.ros_camera_info_topic = cam['camera_info_topic']

        data = cfg['calibration_data']
        self.rotate_image_180 = data['rotate_image_180']
        self.dump_file_name   = data['output_file_name']

        setup = data['data_collection_setup']
        self.max_angle = setup[0]
        self.radius    = [setup[3], setup[4]]

        base_dir = str(pathlib.Path(
            os.path.join(os.path.dirname(__file__), "../calibration_data")
        ).resolve())

        use_existing = data.get('use_existing_data', False)

        if use_existing:
            # ── existing data mode: pick a run folder ─────────────────────────
            existing_runs = sorted(
                [d for d in os.listdir(base_dir)
                 if d.isdigit()
                 and os.path.isdir(os.path.join(base_dir, d, "images"))
                 and len(os.listdir(os.path.join(base_dir, d, "images"))) > 0],
                key=int
            )
            if not existing_runs:
                raise RuntimeError(f"No existing run folders found in {base_dir}")

            print("\nExisting calibration runs:")
            for r in existing_runs:
                n_imgs = len(os.listdir(os.path.join(base_dir, r, "images")))
                print(f"  [{r}]  {n_imgs} images  →  {os.path.join(base_dir, r)}")
            choice = input(f"Choose run [{existing_runs[0]}–{existing_runs[-1]}]: ").strip()
            run_num = int(choice) if choice.isdigit() and choice in existing_runs else int(existing_runs[-1])
            print(f"  → Using run {run_num}\n")
            self.use_existing_data = True
            self.n_cycle = self.n_pose_per_cycle = 0   # not used
        else:
            _cfg_cycles     = int(setup[1])
            _cfg_per_cycle  = int(setup[2])
            _cfg_total      = 1 + _cfg_cycles * _cfg_per_cycle
            _PRESETS = {
                '0': (_cfg_total,  _cfg_cycles, _cfg_per_cycle),
                '1': (  10,  1,  9),
                '2': (  30,  3, 10),
                '3': (  50,  5, 10),
                '4': ( 100,  5, 20),
                '5': ( 300, 15, 20),
                '6': ( 500, 25, 20),
            }
            print("\nSelect number of calibration poses:")
            print(f"  [0]  ~{_cfg_total:>4}  (from config: cycles={_cfg_cycles}, per_cycle={_cfg_per_cycle})")
            for k, (n, c, p) in list(_PRESETS.items())[1:]:
                print(f"  [{k}]  ~{n:>4}  (cycles={c}, per_cycle={p})")
            choice = input("Choice [0-6]: ").strip()
            if choice not in _PRESETS:
                print(f"{ORN}Invalid choice — using config default.{RST}")
                self.n_cycle, self.n_pose_per_cycle = _cfg_cycles, _cfg_per_cycle
            else:
                _, self.n_cycle, self.n_pose_per_cycle = _PRESETS[choice]
            total = 1 + self.n_cycle * self.n_pose_per_cycle
            print(f"  → {total} poses selected.\n")

            # find next unused numbered run directory
            run_num = 1
            while os.path.isdir(os.path.join(base_dir, str(run_num))) and \
                  os.path.isdir(os.path.join(base_dir, str(run_num), "images")) and \
                  len(os.listdir(os.path.join(base_dir, str(run_num), "images"))) > 0:
                run_num += 1
            self.use_existing_data = False

        run_dir = os.path.join(base_dir, str(run_num))
        self.calibration_data_dir_abs = run_dir
        self.images_dir_abs      = os.path.join(run_dir, "images")
        self.images_dir_relative = os.path.join("../calibration_data", str(run_num), "images")

        cfg['calibration_data']['data_relative_dir'] = os.path.join(
            "../calibration_data", str(run_num))
        os.makedirs(self.images_dir_abs, exist_ok=True)

        print(f"Robot: {self.robot_name}  |  topic: {self.ros_image_topic}")
        print(f"Data dir: {self.calibration_data_dir_abs}  [run {run_num}]")

    def _setup_camera(self):
        self.cv_bridge         = CvBridge()
        self._latest_image_msg = None
        self._camera_matrix    = None
        self._dist_coeffs      = None
        self._markers_pub      = self.create_publisher(Image, 'markers_image', 1)
        self._tf_broadcaster   = StaticTransformBroadcaster(self)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._is_compressed = self.ros_image_topic.endswith('/compressed')
        msg_type = CompressedImage if self._is_compressed else Image
        self.create_subscription(msg_type, self.ros_image_topic, self._image_callback, qos)
        self.create_subscription(CameraInfo, self.ros_camera_info_topic, self._info_callback, 1)

    def _setup_target(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.target_aruco_dict)
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod        = cv2.aruco.CORNER_REFINE_CONTOUR
        self.detector_params.cornerRefinementMinAccuracy   = 0.00001
        self.detector_params.cornerRefinementMaxIterations = 1000

        if self.target_type == 'charuco':
            self.charuco_board = cv2.aruco.CharucoBoard_create(
                squaresX=self.target_size[0], squaresY=self.target_size[1],
                squareLength=self.checker_length, markerLength=self.marker_length,
                dictionary=self.aruco_dict,
            )
            try:
                self.charuco_board.setLegacyPattern(self.legacy_pattern)
            except Exception:
                pass

    def _image_callback(self, msg):
        self._latest_image_msg = msg

    def _info_callback(self, msg):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k).reshape(3, 3)
            self._dist_coeffs   = np.array(msg.d).reshape(-1, 1)

    def _publish_charuco_tf(self, gray):
        if self._camera_matrix is None:
            return
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict,
                                                   parameters=self.detector_params)
        if ids is None or len(ids) < 6:
            return
        _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.charuco_board)
        if ch_ids is None or len(ch_ids) < 6:
            return
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            ch_corners, ch_ids, self.charuco_board,
            self._camera_matrix, self._dist_coeffs, None, None)
        if not ok:
            return
        rot_mat, _ = cv2.Rodrigues(rvec)
        q = R.from_matrix(rot_mat).as_quat()

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id  = 'charuco_target'
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self._tf_broadcaster.sendTransform(t)

    def _get_image(self, timeout=10.0):
        deadline = time.time() + timeout
        while self._latest_image_msg is None and time.time() < deadline:
            time.sleep(0.05)
        if self._latest_image_msg is None:
            raise TimeoutError(f"No image on '{self.ros_image_topic}' after {timeout}s")
        return self._latest_image_msg

    def get_image(self):
        msg = self._get_image()
        if self._is_compressed:
            cv_image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        else:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        if self.rotate_image_180:
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) if len(cv_image.shape) == 3 else cv_image
        blurred = cv2.GaussianBlur(gray, (self.blur[0], self.blur[0]), self.blur[1])
        rgb_msg = self.cv_bridge.cv2_to_imgmsg(cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR), encoding="bgr8")
        return rgb_msg, blurred

    def _publish_markers(self, rgb_msg, corners, ids):
        img = self.cv_bridge.imgmsg_to_cv2(rgb_msg)
        for i, c in enumerate(corners):
            pts = c[0].astype(int)
            cv2.putText(img, f"ID:{ids.flat[i]}", tuple(pts[0]-[0,10]),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 2)
            for k in range(4):
                cv2.line(img, tuple(pts[k]), tuple(pts[(k+1)%4]), (0,255,0), 2)
        self._markers_pub.publish(self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8"))

    def wait_for_charuco(self):
        while True:
            rgb, gray = self.get_image()
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
            if ids is not None and len(ids) >= 6:
                _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.charuco_board)
                if ch_ids is not None and len(ch_ids) >= 6:
                    print(f'\033[32mFound {len(ids)} arucos.\033[0m')
                    self._publish_markers(rgb, corners, ids)
                    self._publish_charuco_tf(gray)
                    return gray
                self._publish_markers(rgb, corners, ids)
                input(f'\033[33mOnly {len(ids)} arucos — adjust pose and press ENTER.\033[0m')

    def save_image(self, name: str, frame: np.ndarray):
        path = os.path.join(self.images_dir_abs, name if name.endswith('.png') else name + '.png')
        cv2.imwrite(path, frame)
        self.get_logger().info(f"Image: {path}")

    def save_json(self):
        path = os.path.join(self.calibration_data_dir_abs, self.dump_file_name + '.json')
        with open(path, 'w') as f:
            json.dump(self.calibration_data_list, f, indent=4)
        self.get_logger().info(f"JSON:  {path}")

    def collect_data(self):
        from aric_camera_calibration.doosan_ros2 import build_calib_poses

        self.robot.move_to_calib_start()

        center = self.robot.get_current_posx()
        print(f"Center: {[round(v,2) for v in center]}")

        poses = build_calib_poses(
            center,
            n_cycles    = self.n_cycle,
            n_per_cycle = self.n_pose_per_cycle,
            xy_mm       = self.radius[0] * 1000 * 0.3,
            z_mm        = (self.radius[1] - self.radius[0]) * 1000 * 0.5,
            tilt_deg    = math.degrees(self.max_angle),
        )
        input(f"{len(poses)} poses ready. Press ENTER to start...")

        from tqdm.auto import tqdm
        for pose in tqdm(poses, desc='Collecting'):
            if not self.robot.move_posx(pose):
                print("move failed, skipping.")
                continue

            time.sleep(1.0)   # let robot fully settle
            gray = self.wait_for_charuco()
            time.sleep(1.0)   # hold before moving to next pose

            ee_tf = self.robot.get_ee_matrix()

            name = f"{self.data_counter}.png"
            self.calibration_data_list[name] = {
                'ee_pose': ee_tf.tolist(),
                'image':   os.path.join(self.images_dir_relative, name),
            }
            self.save_image(name, gray)
            self.save_json()
            self.data_counter += 1

        self.data_collected = (self.data_counter == len(self.calibration_data_list))


def main():
    rclpy.init()
    node = executor = executor_thread = None
    calibration_config = None
    try:
        node = CameraCalibrationDataCollection()
        calibration_config = node.calibration_config
        if not node.use_existing_data:
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            executor_thread = threading.Thread(target=executor.spin, daemon=True)
            executor_thread.start()
            time.sleep(0.5)
            node.collect_data()
    finally:
        if executor_thread:
            executor_thread.join(timeout=2.0)
        if node:
            node.destroy_node()
        rclpy.shutdown()

    # ── Calibration (runs after ROS2 is shut down — no node needed) ──────────
    if calibration_config is None:
        print("[ERROR] No config available — cannot calibrate.")
        return

    from aric_camera_calibration.aric_camera_calibration import CameraCalibrator
    calibrator = CameraCalibrator(calibration_config)

    print("\nIntrinsic calibration source:")
    print("  [0]  Load from RealSense /camera_info topic (default)")
    print("  [1]  Compute from collected images (CharuCo calibration)")
    intr = input("Choice [0/1]: ").strip()
    if intr == '1':
        calibrator.calibrate_camera_intrinsics()
    else:
        calibrator.load_realsense_intrinsics()

    print("\nExtrinsic calibration method:")
    print("  [0]  ARIC solver (iterative, default)")
    print("  [1]  OpenCV Tsai")
    print("  [2]  OpenCV Park")
    print("  [3]  OpenCV Horaud")
    print("  [4]  OpenCV Andreff")
    print("  [5]  OpenCV Daniilidis")
    ext = input("Choice [0-5]: ").strip()
    _EXT_MAP = {
        '0': 'ARIC',
        '1': 'TSAI',
        '2': 'PARK',
        '3': 'HORAUD',
        '4': 'ANDREFF',
        '5': 'DANIILIDIS',
    }
    ext_method = _EXT_MAP.get(ext, 'ARIC')
    calibrator.calibrate_camera_extrinsics(method=ext_method)
    print(f"\nResults:\n  {calibrator.calibration_results_txt}"
          f"\n  {calibrator.calibration_results_json}")


if __name__ == "__main__":
    main()
