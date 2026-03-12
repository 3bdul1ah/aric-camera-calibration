#!/usr/bin/python3
import json
import os
import pathlib
import threading
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from aric_camera_calibration.utils import ARUCO_DICT, BLU, GRN, YLW, RST


class CameraCalibrator:

    def __init__(self, calibration_config: dict):
        self.calibration_config = calibration_config

        self.datadir = str(pathlib.Path(
            os.path.join(
                os.path.dirname(__file__),
                calibration_config['calibration_data']['data_relative_dir'],
                "images"
            )
        ).resolve())

        pose_file = os.path.join(
            self.datadir[:self.datadir.find('images')],
            calibration_config['calibration_data']['output_file_name']
        )
        if not pose_file.endswith('.json'):
            pose_file += '.json'
        with open(pose_file) as f:
            self.pose_data = json.load(f)

        tgt = calibration_config['calibration_target']
        self.markers_dict           = ARUCO_DICT[tgt['aruco_dict']]
        self.charuco_board_size     = tgt['size']
        self.charuco_checker_length = tgt['checker_length']
        self.charuco_marker_length  = tgt['marker_length']
        self.charuco_legacy_pattern = tgt['legacy_pattern']

        self.images = np.array(
            [os.path.join(self.datadir, f) for f in os.listdir(self.datadir) if f.endswith(".png")]
        )
        self.images = self.images[
            np.argsort([int(os.path.splitext(os.path.basename(p))[0]) for p in self.images])
        ]
        self.image_size = cv2.imread(self.images[0]).shape[:2]

        self.aruco_dict    = cv2.aruco.getPredefinedDictionary(self.markers_dict)
        self.charuco_board = cv2.aruco.CharucoBoard_create(
            squaresX=self.charuco_board_size[0],
            squaresY=self.charuco_board_size[1],
            squareLength=self.charuco_checker_length,
            markerLength=self.charuco_marker_length,
            dictionary=self.aruco_dict,
        )
        try:
            self.charuco_board.setLegacyPattern(self.charuco_legacy_pattern)
        except Exception:
            pass

        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod      = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector_params.cornerRefinementMinAccuracy  = 0.0001
        self.detector_params.cornerRefinementMaxIterations = 10000

        self.allCharucoCorners   = []
        self.allCharucoIds       = []
        self.allMarkerCorners    = []
        self.allMarkerIds        = []
        self.valid_indices       = []
        self.camera_matrix       = None
        self.distortion_coeffs   = None
        self.reprojection_error  = None
        self.rotation_vectors    = []
        self.translation_vectors = []
        self.cam_T_target        = []
        self.base_T_tcp          = []
        self.tcp_T_cam           = None
        self.base_T_target       = None

        out_dir = str(pathlib.Path(os.path.join(self.datadir, '..')).resolve())
        self.calibration_results_txt  = os.path.join(out_dir, 'calibration_results.txt')
        self.calibration_results_json = os.path.join(out_dir, 'calibration_results.json')

    def read_charuco_board(self):
        print(BLU + "READING CHARUCO BOARD:" + RST)
        for idx, im in enumerate(self.images):
            gray = cv2.cvtColor(cv2.imread(im), cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
            if ids is None or len(ids) == 0:
                print(f"  [{idx}] no markers — skipping.")
                continue
            _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.charuco_board)
            if ch_ids is None or len(ch_ids) < 6:
                print(f"  [{idx}] only {0 if ch_ids is None else len(ch_ids)} corners — skipping.")
                continue
            self.allCharucoCorners.append(ch_corners)
            self.allCharucoIds.append(ch_ids)
            self.allMarkerCorners.append(corners)
            self.allMarkerIds.append(ids)
            self.valid_indices.append(idx)
        print(f"  {len(self.valid_indices)}/{len(self.images)} images usable.")

    def load_realsense_intrinsics(self, topic=None, timeout=5.0):
        import rclpy
        from sensor_msgs.msg import CameraInfo
        if topic is None:
            topic = self.calibration_config['camera']['camera_info_topic']

        rclpy.init()
        node = rclpy.create_node('_camera_info_reader')
        got  = threading.Event()

        def cb(msg):
            self.camera_matrix     = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d).reshape(-1, 1)
            got.set()

        node.create_subscription(CameraInfo, topic, cb, 1)
        deadline = time.time() + timeout
        while not got.is_set() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()

        if not got.is_set():
            raise TimeoutError(f"No camera_info on '{topic}' after {timeout}s")
        print(f"[RealSense] intrinsics from {topic}\n  K:\n{self.camera_matrix}\n  D: {self.distortion_coeffs.T}")

    def calibrate_camera_intrinsics(self, flags=0):
        if not self.allCharucoCorners:
            self.read_charuco_board()
        print("+------------------------------+")
        print(f'|{BLU} INTRINSIC CAMERA CALIBRATION {RST}|')
        print("+------------------------------+")

        h, w = self.image_size
        K0 = np.array([[0., 0., w / 2.],
                        [0., 0., h / 2.],
                        [0., 0., 1.]])

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    10000, 1e-9)

        (
            self.reprojection_error,
            self.camera_matrix,
            self.distortion_coeffs,
            self.rotation_vectors,
            self.translation_vectors,
            _stdInt, _stdExt, _perView,
        ) = cv2.aruco.calibrateCameraCharucoExtended(
            charucoCorners=self.allCharucoCorners,
            charucoIds=self.allCharucoIds,
            board=self.charuco_board,
            imageSize=(w, h),
            cameraMatrix=K0,
            distCoeffs=np.zeros((5, 1)),
            flags=flags,
            criteria=criteria,
        )
        np.set_printoptions(suppress=True)
        print(f'* {GRN}Reprojection Error:{RST} {self.reprojection_error}')
        print(f'* {GRN}Camera Matrix{RST}')
        for row in self.camera_matrix:
            print(f'  {row}')
        print(f'* {GRN}Distortion Coefficients{RST}')
        print("  " + str([c[0].tolist() for c in self.distortion_coeffs]))

        with open(self.calibration_results_txt, 'w') as f:
            f.write("+------------------------------+\n")
            f.write("| INTRINSIC CAMERA CALIBRATION |\n")
            f.write("+------------------------------+\n")
            f.write(f"* Reprojection Error: {self.reprojection_error}\n")
            f.write("* Camera Matrix\n")
            for row in self.camera_matrix:
                f.write(f"  {row}\n")
            f.write("* Distortion Coefficients\n")
            f.write("  " + str([c[0].tolist() for c in self.distortion_coeffs]) + "\n")

    def _get_cam_T_target_poses(self):
        if not self.allCharucoCorners:
            self.read_charuco_board()
        if self.rotation_vectors:
            rvecs, tvecs = self.rotation_vectors, self.translation_vectors
        else:
            rvecs, tvecs = [], []
            for corners, ids in zip(self.allCharucoCorners, self.allCharucoIds):
                _, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    corners, ids, self.charuco_board,
                    self.camera_matrix, self.distortion_coeffs,
                    np.empty(1), np.empty(1)
                )
                rvecs.append(rvec)
                tvecs.append(tvec)
            # store so metrics can use them
            self.rotation_vectors    = rvecs
            self.translation_vectors = tvecs
        for rvec, tvec in zip(rvecs, tvecs):
            p   = np.array(tvec).reshape(3, -1)
            rot = Rotation.from_rotvec(np.array(rvec).reshape(3)).as_matrix()
            self.cam_T_target.append(np.vstack([np.c_[rot, p], [0, 0, 0, 1]]))

    def _get_base_T_tcp_poses(self):
        sample_key = next(iter(self.pose_data))
        if sample_key.endswith('.png'):
            # New format: key IS the image filename — direct O(1) lookup, no ordering bug.
            self.base_T_tcp = [
                np.array(self.pose_data[os.path.basename(self.images[i])]['ee_pose'])
                for i in self.valid_indices
            ]
        else:
            # Legacy format: data_sample_N — sort numerically to match image 0.png…N.png.
            sorted_keys = sorted(
                self.pose_data.keys(),
                key=lambda k: int(k.split('_')[-1])
            )
            all_poses = [self.pose_data[k]['ee_pose'] for k in sorted_keys]
            self.base_T_tcp = [all_poses[i] for i in self.valid_indices]
        print(f"  EE poses matched: {len(self.base_T_tcp)}")

    @staticmethod
    def _print_tf(label, tf):
        tf = np.array(tf)
        print(f'* {GRN}{label}{RST}')
        for row in tf:
            print(f'  {row}')
        t = tf[:3, 3]
        r = Rotation.from_matrix(tf[:3, :3])
        ang  = r.as_euler('xyz')
        angd = r.as_euler('xyz', degrees=True)
        print(f'* {GRN}Translation{RST}  {t}')
        print(f'* {GRN}Euler (xyz){RST}')
        print(f'  {YLW}rad{RST} X:{ang[0]:.4f}  Y:{ang[1]:.4f}  Z:{ang[2]:.4f}')
        print(f'  {YLW}deg{RST} X:{angd[0]:.4f} Y:{angd[1]:.4f} Z:{angd[2]:.4f}')
        print()
        return t, tf[:3, :3], ang, angd

    def solve_transform(self, X, Y):
        def ralign(X, Y):
            m, n = X.shape
            mx, my = X.mean(1), Y.mean(1)
            Xc  = X - np.tile(mx, (n, 1)).T
            Yc  = Y - np.tile(my, (n, 1)).T
            sx  = np.mean(np.sum(Xc * Xc, 0))
            Sxy = np.dot(Yc, Xc.T) / n
            U, D, V = np.linalg.svd(Sxy, full_matrices=True, compute_uv=True)
            V = V.T.copy()
            S = np.eye(m)
            if np.ndim(Sxy) > (m - 1) and np.linalg.det(Sxy) < 0:
                S[m-1, m-1] = -1
            R = np.dot(np.dot(U, S), V.T)
            t = my - np.trace(np.dot(np.diag(D), S)) / sx * np.dot(R, mx)
            return R, t
        R, T = ralign(X[:3, :], Y[:3, :])
        H = np.eye(4)
        H[:3, :3] = R
        H[:3,  3] = T
        return H

    def _solve_aric(self, H_TBs, H_ACs, initial_tf):
        N, L = 30, 0.5
        X_A = np.zeros([4, 3 * N])
        X_A[0, 0:N]     = np.linspace(0, L, N + 1)[1:]
        X_A[1, N:2*N]   = np.linspace(0, L, N + 1)[1:]
        X_A[2, 2*N:3*N] = np.linspace(0, L, N + 1)[1:]
        X_A[3, :]        = 1

        H_CT    = initial_tf
        X_B_old = None
        for it in range(2000):
            X_Bj = X_Aj = None
            for i, (H_TB, H_AC) in enumerate(zip(H_TBs, H_ACs)):
                X_Bi = H_TB @ H_CT @ H_AC @ X_A
                X_Bj = X_Bi if i == 0 else np.append(X_Bj, X_Bi, axis=1)
                X_Aj = X_A  if i == 0 else np.append(X_Aj, X_A,  axis=1)
            H_AB = self.solve_transform(X_Aj, X_Bj)
            X_B  = H_AB @ X_A
            if X_B_old is not None:
                rmse = np.average(np.linalg.norm((X_B - X_B_old)[:3, :], axis=0))
                if it % 200 == 0:
                    print(f'  RMSE: {rmse:.2e}')
                if rmse < 1e-15:
                    print("Calibration converged!")
                    break
            X_B_old = np.copy(X_B)
            X_Tj = X_Cj = None
            for i, (H_TB, H_AC) in enumerate(zip(H_TBs, H_ACs)):
                X_Ti = np.linalg.inv(H_TB) @ X_B
                X_Ci = H_AC @ X_A
                X_Tj = X_Ti if i == 0 else np.append(X_Tj, X_Ti, axis=1)
                X_Cj = X_Ci if i == 0 else np.append(X_Cj, X_Ci, axis=1)
            H_CT = self.solve_transform(X_Cj, X_Tj)
        return H_CT, H_AB

    def compute_metrics(self):
        """Compute reprojection error and target-position consistency."""
        K  = self.camera_matrix
        D  = self.distortion_coeffs.flatten()   # (5,) expected by projectPoints

        sq   = self.charuco_checker_length       # squareLength in metres
        sx   = self.charuco_board_size[0]        # squaresX

        def _board_pts(ids_flat):
            """Return (N,1,3) float32 3-D board positions for the given corner IDs.

            Charuco corner id k is at column (k % (sx-1)) and row (k // (sx-1))
            in 0-based board coordinates → world position ((col+1)*sq, (row+1)*sq, 0).
            """
            pts = np.array(
                [[(int(k) % (sx - 1) + 1) * sq,
                  (int(k) // (sx - 1) + 1) * sq,
                  0.0] for k in ids_flat],
                dtype=np.float32,
            ).reshape(-1, 1, 3)
            return pts

        # ── 1. RMS reprojection error (per pose) ─────────────────────────────
        sq_errs, n_pts = [], []
        per_pose_rms   = []
        for rvec, tvec, corners_2d, ids in zip(
                self.rotation_vectors, self.translation_vectors,
                self.allCharucoCorners, self.allCharucoIds):
            if rvec is None or tvec is None:
                continue
            obj_pts = _board_pts(ids.flatten())
            proj, _ = cv2.projectPoints(obj_pts,
                                        np.array(rvec).reshape(3, 1),
                                        np.array(tvec).reshape(3, 1),
                                        K, D)
            proj = proj.reshape(-1, 2)
            det  = corners_2d.reshape(-1, 2)
            errs = np.linalg.norm(proj - det, axis=1)
            rms  = float(np.sqrt(np.mean(errs ** 2)))
            per_pose_rms.append(rms)
            sq_errs.append(np.sum(errs ** 2))
            n_pts.append(len(errs))

        rms_total = float(np.sqrt(sum(sq_errs) / sum(n_pts))) if n_pts else float('nan')

        # ── 2. Target-position consistency (base_T_target std dev) ────────────
        base_T_targets = []
        for base_T_tcp, cam_T_target in zip(self.base_T_tcp, self.cam_T_target):
            BT = np.array(base_T_tcp) @ self.tcp_T_cam @ cam_T_target
            base_T_targets.append(BT)
        trans = np.array([T[:3, 3] for T in base_T_targets])   # (N,3) in metres
        trans_mean_m = trans.mean(axis=0)
        trans_std_m  = trans.std(axis=0)
        trans_std_mm = trans_std_m * 1000.0

        rots = [Rotation.from_matrix(T[:3, :3]) for T in base_T_targets]
        mean_rot = Rotation.mean(Rotation.concatenate(rots))
        ang_diffs_deg = np.array([
            float(np.degrees((r * mean_rot.inv()).magnitude())) for r in rots
        ])
        rot_std_deg = float(ang_diffs_deg.std())

        # ── print ─────────────────────────────────────────────────────────────
        print(f'\n{"─"*48}')
        print(f'  CALIBRATION QUALITY METRICS')
        print(f'{"─"*48}')
        print(f'  Reprojection error (RMS, px)   : {GRN}{rms_total:.4f}{RST}  px')
        print(f'  Target pos std  (X,Y,Z)  mm    : '
              f'{YLW}{trans_std_mm[0]:.2f}  {trans_std_mm[1]:.2f}  {trans_std_mm[2]:.2f}{RST}')
        print(f'  Target rot std (angular)  deg  : {YLW}{rot_std_deg:.4f}{RST}')
        print(f'  Poses used                     : {len(per_pose_rms)}')
        print(f'{"─"*48}\n')

        return {
            'reprojection_error_rms_px':   rms_total,
            'per_pose_reprojection_rms_px': per_pose_rms,
            'target_position_mean_m':      trans_mean_m.tolist(),
            'target_position_std_mm':      trans_std_mm.tolist(),
            'target_rotation_std_deg':     rot_std_deg,
            'n_poses':                     len(per_pose_rms),
        }

    def calibrate_camera_extrinsics(self, method='OPENCV'):
        self._get_cam_T_target_poses()
        self._get_base_T_tcp_poses()

        print("+------------------------------+")
        print(f'|{BLU} EXTRINSIC CAMERA CALIBRATION {RST}|')
        print("+------------------------------+")

        _OPENCV_METHODS = {
            'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
            'PARK':       cv2.CALIB_HAND_EYE_PARK,
            'HORAUD':     cv2.CALIB_HAND_EYE_HORAUD,
            'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
            'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
        }

        if method == 'ARIC':
            init_t = np.array([0.0, 0.1, 0.1]).reshape(3, -1)
            init_R = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], dtype=float)
            init_tf = np.vstack([np.c_[init_R, init_t], [0, 0, 0, 1]])
            self.tcp_T_cam, self.base_T_target = self._solve_aric(
                self.base_T_tcp, self.cam_T_target, init_tf
            )
        elif method in _OPENCV_METHODS:
            R_C = np.array([T[:3, :3] for T in self.cam_T_target])
            t_C = np.array([T[:3, 3]  for T in self.cam_T_target])
            R_B = np.array([np.array(T)[:3, :3] for T in self.base_T_tcp])
            t_B = np.array([np.array(T)[:3, 3]  for T in self.base_T_tcp])
            cv_method = _OPENCV_METHODS[method]
            print(f"  Using OpenCV {method}")
            rot, trans = cv2.calibrateHandEye(R_B, t_B, R_C, t_C, method=cv_method)
            self.tcp_T_cam = np.vstack([np.c_[rot, trans], [0, 0, 0, 1]])

            # Derive base_T_target: average over all pose pairs
            bt_list = [
                np.array(self.base_T_tcp[i]) @ self.tcp_T_cam @ self.cam_T_target[i]
                for i in range(len(self.cam_T_target))
            ]
            mean_t = np.mean([T[:3, 3] for T in bt_list], axis=0)
            mean_R = Rotation.mean(
                Rotation.from_matrix(np.array([T[:3, :3] for T in bt_list]))
            ).as_matrix()
            self.base_T_target = np.vstack([np.c_[mean_R, mean_t.reshape(3, 1)], [0, 0, 0, 1]])

        has_bt = self.base_T_target is not None and np.array(self.base_T_target).size > 0

        t_ct, R_ct, ang_ct, angd_ct = self._print_tf('tcp_T_cam', self.tcp_T_cam)
        if has_bt:
            t_bt, R_bt, ang_bt, angd_bt = self._print_tf('base_T_target (board origin)', self.base_T_target)

        metrics = self.compute_metrics()

        def write_tf(f, label, tf, t, ang, angd):
            f.write(f"* {label}\n")
            for row in np.array(tf):
                f.write(f"  {row}\n")
            f.write(f"* Translation  {t}\n")
            f.write(f"* Euler xyz (rad) X:{ang[0]}  Y:{ang[1]}  Z:{ang[2]}\n")
            f.write(f"* Euler xyz (deg) X:{angd[0]} Y:{angd[1]} Z:{angd[2]}\n")

        with open(self.calibration_results_txt, 'a') as f:
            f.write("| EXTRINSIC CAMERA CALIBRATION |\n")
            write_tf(f, 'tcp_T_cam', self.tcp_T_cam, t_ct, ang_ct, angd_ct)
            if has_bt:
                write_tf(f, 'base_T_target (board origin)', self.base_T_target, t_bt, ang_bt, angd_bt)
            f.write("| QUALITY METRICS |\n")
            f.write(f"* Reprojection error RMS (px)  : {metrics['reprojection_error_rms_px']:.4f}\n")
            f.write(f"* Target pos std (X,Y,Z)  mm   : {[round(v,3) for v in metrics['target_position_std_mm']]}\n")
            f.write(f"* Target rot std (deg)         : {metrics['target_rotation_std_deg']:.4f}\n")
            f.write(f"* Poses used                   : {metrics['n_poses']}\n")

        results = {
            'camera_matrix':    self.camera_matrix.tolist(),
            'distortion_coeffs': self.distortion_coeffs.tolist(),
            'tcp_T_cam': {
                'T':               self.tcp_T_cam.tolist(),
                'translation_vec': t_ct.tolist(),
                'rotation_mat':    R_ct.tolist(),
                'euler_angles_rad': ang_ct.tolist(),
                'euler_angles_deg': angd_ct.tolist(),
            },
            'quality_metrics': metrics,
        }
        if has_bt:
            results['base_T_target'] = {
                'T':               np.array(self.base_T_target).tolist(),
                'translation_vec': t_bt.tolist(),
                'rotation_mat':    R_bt.tolist(),
                'euler_angles_rad': ang_bt.tolist(),
                'euler_angles_deg': angd_bt.tolist(),
            }
        with open(self.calibration_results_json, 'w') as f:
            json.dump(results, f, ensure_ascii=True, indent=4)
