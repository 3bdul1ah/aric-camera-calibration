#!/usr/bin/python3
"""
Live CharuCo detection viewer. No robot needed.

Usage:
    ros2 run aric_camera_calibration charuco_check
"""
import json
import os
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from aric_camera_calibration.utils import ARUCO_DICT

CONFIG = os.path.join(os.path.dirname(__file__), "../config/calibration_config.json")
WINDOW = "CharuCo Check  |  q = quit"


class CharucoViewer(Node):

    def __init__(self, cfg: dict):
        super().__init__('charuco_check')

        tgt  = cfg['calibration_target']
        cam  = cfg['camera']
        data = cfg['calibration_data']

        aruco_enum = ARUCO_DICT[tgt['aruco_dict']]
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_enum)

        self.sqx    = tgt['size'][0]
        self.sqy    = tgt['size'][1]
        self.sq_len = tgt['checker_length']

        self.board = cv2.aruco.CharucoBoard_create(
            squaresX=self.sqx, squaresY=self.sqy,
            squareLength=self.sq_len, markerLength=tgt['marker_length'],
            dictionary=self.aruco_dict,
        )
        try:
            self.board.setLegacyPattern(tgt['legacy_pattern'])
        except Exception:
            pass

        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod       = cv2.aruco.CORNER_REFINE_CONTOUR
        self.params.cornerRefinementMinAccuracy   = 0.00001
        self.params.cornerRefinementMaxIterations = 1000

        self.axis_len   = self.sq_len * 2
        self.rotate_180 = data.get('rotate_image_180', False)
        self.blur_k     = tgt['blur'][0]
        self.blur_s     = tgt['blur'][1]

        self.bridge        = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs   = None
        self._latest_frame = None
        self._lock         = threading.Lock()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        image_topic = cam['image_topic']
        info_topic  = cam['camera_info_topic']

        if image_topic.endswith('/compressed'):
            self.create_subscription(CompressedImage, image_topic,
                                     self._img_cb_compressed, qos)
        else:
            self.create_subscription(Image, image_topic,
                                     self._img_cb_raw, qos)

        self.create_subscription(CameraInfo, info_topic, self._info_cb, 1)
        self.get_logger().info(f"image : {image_topic}")
        self.get_logger().info(f"info  : {info_topic}")

    def _info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d).reshape(-1, 1)
            self.get_logger().info(
                f"intrinsics received  fx={self.camera_matrix[0,0]:.1f}")

    def _img_cb_compressed(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                             cv2.IMREAD_COLOR)
        with self._lock:
            self._latest_frame = frame

    def _img_cb_raw(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock:
            self._latest_frame = frame

    def get_frame(self):
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def process(self, frame: np.ndarray) -> np.ndarray:
        if self.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (self.blur_k, self.blur_k), self.blur_s)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.params)

        out = frame.copy()

        if ids is None or len(ids) == 0:
            cv2.putText(out, "No markers detected", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
            return out

        cv2.aruco.drawDetectedMarkers(out, corners, ids)

        _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board)

        if ch_ids is None or len(ch_ids) < 6:
            cv2.putText(out,
                        f"Markers: {len(ids)}  |  need more CharuCo corners",
                        (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 165, 255), 2)
            return out

        cv2.aruco.drawDetectedCornersCharuco(out, ch_corners, ch_ids, (0, 255, 0))

        if self.camera_matrix is None:
            cv2.putText(out, "Waiting for camera_info ...", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
            return out

        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            ch_corners, ch_ids, self.board,
            self.camera_matrix, self.dist_coeffs, None, None)
        if not ok:
            cv2.putText(out, "Pose estimation failed", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return out

        # axes at board origin (0,0)
        self._draw_axes(out, rvec, tvec)
        origin_px = self._project(np.zeros((1, 3)), rvec, tvec)
        cv2.circle(out, origin_px, 10, (255, 255, 255), 3)
        cv2.putText(out, "origin", (origin_px[0] + 14, origin_px[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        dist_cm = float(np.linalg.norm(tvec)) * 100
        cv2.putText(out,
                    f"Corners: {len(ch_ids)}  |  dist: {dist_cm:.1f} cm",
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        return out

    def _draw_axes(self, img, rvec, tvec):
        try:
            cv2.aruco.drawAxis(img, self.camera_matrix, self.dist_coeffs,
                               rvec, tvec, self.axis_len)
        except AttributeError:
            cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, self.axis_len)

    def _project(self, pts_3d, rvec, tvec):
        px, _ = cv2.projectPoints(pts_3d.astype(np.float64),
                                  rvec, tvec,
                                  self.camera_matrix, self.dist_coeffs)
        return int(px[0][0][0]), int(px[0][0][1])


def main():
    with open(CONFIG) as f:
        cfg = json.load(f)

    rclpy.init()
    node     = CharucoViewer(cfg)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("[CharucoCheck] Waiting for camera feed ...  press q to quit.")
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1280, 720)

    while rclpy.ok():
        frame = node.get_frame()
        if frame is not None:
            out = node.process(frame)
            cv2.imshow(WINDOW, out)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
