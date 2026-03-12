#!/usr/bin/env python3
"""
Publish tcp_T_cam as a static TF.

Usage:
  ros2 run aric_camera_calibration publish_tcp_T_cam_tf -- \
      --x -0.037205 --y -0.105527 --z 0.039489 \
      --roll -4.5184 --pitch -0.0931 --yaw 0.5632

  Translation in metres, rotation in degrees (Euler XYZ).
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R


class TcpCamTFPublisher(Node):
    def __init__(self, x, y, z, roll_deg, pitch_deg, yaw_deg,
                 parent_frame, child_frame):
        super().__init__('tcp_T_cam_publisher')

        roll  = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw   = math.radians(yaw_deg)
        q = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()  # [x,y,z,w]

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id  = child_frame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(t)

        self.get_logger().info(
            f'Publishing static TF: {parent_frame} -> {child_frame}\n'
            f'  Translation : [{x:.6f}, {y:.6f}, {z:.6f}] m\n'
            f'  Euler (deg) : roll={roll_deg:.4f}  pitch={pitch_deg:.4f}  yaw={yaw_deg:.4f}\n'
            f'  Quaternion  : [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]'
        )


def main():
    parser = argparse.ArgumentParser(description='Publish tcp_T_cam as a static TF')
    parser.add_argument('--x', type=float, required=True, help='X translation (m)')
    parser.add_argument('--y', type=float, required=True, help='Y translation (m)')
    parser.add_argument('--z', type=float, required=True, help='Z translation (m)')
    parser.add_argument('--roll',  type=float, required=True, help='Roll around X (deg)')
    parser.add_argument('--pitch', type=float, required=True, help='Pitch around Y (deg)')
    parser.add_argument('--yaw',   type=float, required=True, help='Yaw around Z (deg)')
    parser.add_argument('--parent-frame', type=str, default='link_6',
                        help='Parent frame (default: link_6)')
    parser.add_argument('--child-frame', type=str, default='camera_color_optical_frame_calibrated',
                        help='Child frame (default: camera_color_optical_frame_calibrated)')
    args = parser.parse_args()

    rclpy.init()
    node = TcpCamTFPublisher(
        args.x, args.y, args.z,
        args.roll, args.pitch, args.yaw,
        args.parent_frame, args.child_frame,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
