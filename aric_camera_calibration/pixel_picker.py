#!/usr/bin/env python3
"""
Click on the camera image to get pixel coords, depth, and 3D position.
Also continuously detects CharuCo board origin (0,0) and shows its 3D coords.

Requires:
  - RealSense camera with align_depth enabled
  - (optional) Robot driver + publish_tcp_T_cam_tf for base-frame coords

Usage:
    ros2 run aric_camera_calibration pixel_picker
"""
import threading

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image

import tf2_ros

WINDOW = "Pixel Picker  |  click = measure  |  q = quit"

COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
BASE_FRAME  = "arm_base_link"
CAM_FRAME   = "camera_color_optical_frame_calibrated" # or camera_color_optical_frame

# Calibrated intrinsics from calibration_data/18/calibration_results.json
CALIB_K = np.array([
    [909.720092, 0.0,        631.430478],
    [0.0,        912.342342, 358.797600],
    [0.0,        0.0,        1.0       ],
])
CALIB_D = np.array([0.136949, -0.417712, -0.000410, -0.000585, 0.341091])

# CharuCo board config (must match calibration_config.json)
CHARUCO_COLS       = 11
CHARUCO_ROWS       = 8
CHECKER_LENGTH_M   = 0.029
MARKER_LENGTH_M    = 0.021
ARUCO_DICT_NAME    = "DICT_6X6_250"
CHARUCO_LEGACY     = True

MEDIAN_RADIUS = 4


def tf_to_matrix(transform):
    t = transform.transform
    T = np.eye(4)
    T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
    T[:3, :3] = R.from_quat(q).as_matrix()
    return T


def make_charuco_board():
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, ARUCO_DICT_NAME))
    board = cv2.aruco.CharucoBoard_create(
        squaresX=CHARUCO_COLS, squaresY=CHARUCO_ROWS,
        squareLength=CHECKER_LENGTH_M, markerLength=MARKER_LENGTH_M,
        dictionary=aruco_dict)
    try:
        board.setLegacyPattern(CHARUCO_LEGACY)
    except Exception:
        pass
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod       = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementMinAccuracy  = 0.0001
    params.cornerRefinementMaxIterations = 10000
    return aruco_dict, board, params


class PixelPicker(Node):

    def __init__(self):
        super().__init__("pixel_picker")
        self.bridge = CvBridge()

        self.K = CALIB_K
        self.D = CALIB_D
        self._color = None
        self._depth = None
        self._lock = threading.Lock()

        self.checker_length = CHECKER_LENGTH_M
        self.aruco_dict, self.board, self.aruco_params = make_charuco_board()

        color_sub = message_filters.Subscriber(
            self, Image, COLOR_TOPIC, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, DEPTH_TOPIC, qos_profile=qos_profile_sensor_data)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=20, slop=0.08)
        self._sync.registerCallback(self._rgbd_cb)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"color: {COLOR_TOPIC}")
        self.get_logger().info(f"depth: {DEPTH_TOPIC}")
        self.get_logger().info(
            f"K (calibrated)  fx={self.K[0,0]:.1f}  fy={self.K[1,1]:.1f}")
        self.get_logger().info(
            f"CharuCo {CHARUCO_COLS}x{CHARUCO_ROWS}  "
            f"checker={CHECKER_LENGTH_M*1000:.0f}mm  marker={MARKER_LENGTH_M*1000:.0f}mm")

    def _rgbd_cb(self, color_msg, depth_msg):
        with self._lock:
            if self._depth is None:
                self.get_logger().info(
                    f"synced RGBD received  "
                    f"color={color_msg.width}x{color_msg.height}  "
                    f"depth={depth_msg.width}x{depth_msg.height}")
            self._color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self._depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

    def get_base_T_cam(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, CAM_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            return tf_to_matrix(tf)
        except Exception:
            return None

    def get_snapshot(self):
        with self._lock:
            if self._color is None or self._depth is None:
                return None
            return self._color.copy(), self._depth.copy()

    def _depth_m(self, depth, u, v):
        h, w = depth.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None

        def to_m(val):
            d = float(val)
            return d / 1000.0 if depth.dtype == np.uint16 else d

        z = to_m(depth[v, u])
        if np.isfinite(z) and z > 0.01:
            return z

        r = MEDIAN_RADIUS
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        patch = depth[v0:v1, u0:u1].astype(np.float32)
        if depth.dtype == np.uint16:
            patch = patch / 1000.0
        valid = patch[(patch > 0.01) & np.isfinite(patch)]
        if valid.size < 3:
            return None
        return float(np.median(valid))

    def pixel_to_3d(self, u, v, depth):
        Z = self._depth_m(depth, u, v)
        if Z is None:
            return None

        if self.D is not None and np.any(self.D != 0):
            pt = cv2.undistortPoints(
                np.array([[[float(u), float(v)]]], dtype=np.float64),
                self.K, self.D)
            X = pt[0][0][0] * Z
            Y = pt[0][0][1] * Z
        else:
            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]
            X = (u - cx) / fx * Z
            Y = (v - cy) / fy * Z

        return np.array([X, Y, Z])

    def measure(self, u, v, depth):
        p_cam = self.pixel_to_3d(u, v, depth)
        if p_cam is None:
            return None

        base_T_cam = self.get_base_T_cam()
        p_base = None
        if base_T_cam is not None:
            p_base = (base_T_cam @ [p_cam[0], p_cam[1], p_cam[2], 1.0])[:3]

        return p_cam[0], p_cam[1], p_cam[2], p_base

    def detect_charuco_origin(self, color, depth):
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        D_col = self.D.reshape(-1, 1)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None or len(ids) < 6:
            return None

        _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board)
        if ch_ids is None or len(ch_ids) < 6:
            return None

        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            ch_corners, ch_ids, self.board,
            self.K, D_col, np.empty(1), np.empty(1))
        if not ok:
            return None

        origin_px, _ = cv2.projectPoints(
            np.array([[0.0, 0.0, 0.0]]), rvec, tvec, self.K, D_col)
        u = int(round(origin_px[0][0][0]))
        v = int(round(origin_px[0][0][1]))

        # 1) CharuCo pose method (rvec/tvec, no depth)
        rot_mat, _ = cv2.Rodrigues(rvec)
        cam_T_target = np.eye(4)
        cam_T_target[:3, :3] = rot_mat
        cam_T_target[:3, 3] = tvec.flatten()
        p_cam_pose = tvec.flatten()

        # 2) Depth Z only at origin pixel
        Z_depth = self._depth_m(depth, u, v)

        # 3) Hybrid: CharuCo X,Y + depth Z
        p_cam_depth = None
        if Z_depth is not None:
            p_cam_depth = np.array([p_cam_pose[0], p_cam_pose[1], Z_depth])

        base_T_cam = self.get_base_T_cam()
        p_base_pose = None
        p_base_depth = None
        if base_T_cam is not None:
            base_T_target = base_T_cam @ cam_T_target
            p_base_pose = base_T_target[:3, 3]
            if p_cam_depth is not None:
                p_base_depth = (base_T_cam @ [*p_cam_depth, 1.0])[:3]

        return {
            "u": u, "v": v,
            "cam_pose": p_cam_pose,
            "cam_depth": p_cam_depth,
            "base_pose": p_base_pose,
            "base_depth": p_base_depth,
            "rvec": rvec, "tvec": tvec,
            "corners": corners, "ids": ids,
            "ch_corners": ch_corners, "ch_ids": ch_ids,
        }


last_overlay = None
last_charuco = None


def on_mouse(event, x, y, flags, param):
    global last_overlay
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    node = param["node"]
    depth = param.get("depth")
    if depth is None:
        return

    result = node.measure(x, y, depth)

    if result is None:
        last_overlay = {"u": x, "v": y, "ok": False}
        node.get_logger().info(f"px=({x}, {y})  no depth")
        return

    X, Y, Z, p_base = result
    last_overlay = {"u": x, "v": y, "ok": True}

    msg = f"px=({x}, {y})  cam=[{X:.4f}, {Y:.4f}, {Z:.4f}]"
    if p_base is not None:
        msg += f"  base=[{p_base[0]:.4f}, {p_base[1]:.4f}, {p_base[2]:.4f}]"
    else:
        msg += "  (no TF)"
    node.get_logger().info(msg)


def draw_charuco(display, charuco, K, D, checker_length):
    cv2.aruco.drawDetectedMarkers(display, charuco["corners"], charuco["ids"])
    cv2.drawFrameAxes(display, K, D.reshape(-1, 1),
                      charuco["rvec"], charuco["tvec"], checker_length * 2)

    u, v = charuco["u"], charuco["v"]
    cv2.circle(display, (u, v), 4, (255, 0, 255), -1)

    y = 25
    cp = charuco["cam_pose"]
    cv2.putText(display, f"pose  cam [{cp[0]:.4f}, {cp[1]:.4f}, {cp[2]:.4f}]",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 0, 255), 2)

    y += 22
    bp = charuco["base_pose"]
    if bp is not None:
        cv2.putText(display, f"pose  base [{bp[0]:.4f}, {bp[1]:.4f}, {bp[2]:.4f}]",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2)
    else:
        cv2.putText(display, "pose  base: no TF",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2)

    y += 22
    cd = charuco["cam_depth"]
    if cd is not None:
        cv2.putText(display, f"depth cam [{cd[0]:.4f}, {cd[1]:.4f}, {cd[2]:.4f}]",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 100, 255), 2)
    else:
        cv2.putText(display, "depth cam: no depth",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2)

    y += 22
    bd = charuco["base_depth"]
    if bd is not None:
        cv2.putText(display, f"depth base [{bd[0]:.4f}, {bd[1]:.4f}, {bd[2]:.4f}]",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 200, 200), 2)
    else:
        cv2.putText(display, "depth base: no depth/TF",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2)


def main():
    global last_overlay, last_charuco

    rclpy.init()
    node = PixelPicker()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    print("\nPixel Picker | click = measure | CharuCo origin auto-detected | q = quit\n")

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1280, 720)
    state = {"node": node, "depth": None}
    cv2.setMouseCallback(WINDOW, on_mouse, state)

    while rclpy.ok():
        snapshot = node.get_snapshot()
        if snapshot is None:
            cv2.waitKey(30)
            continue

        frame, depth = snapshot
        state["depth"] = depth

        charuco = node.detect_charuco_origin(frame, depth)
        if charuco is not None:
            last_charuco = charuco

        display = frame.copy()

        if last_charuco is not None:
            draw_charuco(display, last_charuco, node.K, node.D, node.checker_length)

        if last_overlay is not None:
            u, v = last_overlay["u"], last_overlay["v"]
            color = (0, 255, 0) if last_overlay["ok"] else (0, 0, 255)
            cv2.circle(display, (u, v), 4, color, -1)

        cv2.imshow(WINDOW, display)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
