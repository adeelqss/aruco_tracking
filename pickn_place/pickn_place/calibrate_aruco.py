#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import numpy as np
import os
import yaml

# TF transformations
from tf_transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    euler_from_quaternion,
    quaternion_from_euler
)


class ArUcoCalibrationNode(Node):
    def __init__(self):
        super().__init__('aruco_calibration_rpy_node')

        # ----------------------------
        # Configuration parameters for calibration
        # ----------------------------
        self.BUFFER_SIZE = 30     # Number of poses to accumulate
        self.IQR_MULTIPLIER = 1   # For outlier removal
        self.calibration_link_frame = 'calibration_link'
        self.base_link_frame = 'base_link'

        # Buffer to hold the incoming PoseStamped messages
        self.pose_buffer = []

        # Flag to track if we've already saved calibration
        self.calibration_done = False

        # TF Tools
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber for the ArUco pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/pickn_place/aruco_pose',
            self.aruco_pose_callback,
            10
        )

        # TF broadcaster (optional if you want real-time marker frame)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Path to your ROS package share or a known directory:
        # Adjust to match your environment:
        # E.g., ~/barns_ws/src/pickn_place/share
        self.ws_src_path = os.path.expanduser('~/barns_ws/src/pickn_place/share')

        self.get_logger().info("ArUco calibration node (RPY version) initialized.")

    def aruco_pose_callback(self, msg: PoseStamped):
        # If calibration is already done, skip processing
        if self.calibration_done:
            return

        # Collect the poses into a buffer
        self.pose_buffer.append(msg)

        # Process the buffer only when we have enough samples
        if len(self.pose_buffer) >= self.BUFFER_SIZE:
            self.process_buffer()
            self.pose_buffer.clear()

    def process_buffer(self):
        """
        Processes the accumulated poses, computes the transform offset,
        and saves the calibration data to a YAML file (only once).
        """
        # If calibration is already done, skip
        if self.calibration_done:
            return

        try:
            # ----------------------------
            # 1) Filter / average the ArUco poses
            # ----------------------------
            positions = np.array([
                [pose.pose.position.x,
                 pose.pose.position.y,
                 pose.pose.position.z]
                for pose in self.pose_buffer
            ])

            orientations = np.array([
                [pose.pose.orientation.x,
                 pose.pose.orientation.y,
                 pose.pose.orientation.z,
                 pose.pose.orientation.w]
                for pose in self.pose_buffer
            ])

            valid_indices = self.filter_outliers_iqr(positions)
            filtered_positions = positions[valid_indices]
            filtered_orientations = orientations[valid_indices]

            if len(filtered_positions) < 1:
                self.get_logger().warning("No valid ArUco data after outlier removal. Skipping calibration.")
                return

            mean_position = filtered_positions.mean(axis=0)
            mean_orientation = filtered_orientations.mean(axis=0)
            norm = np.linalg.norm(mean_orientation)
            if norm < 1e-9:
                self.get_logger().error("Cannot normalize orientation, norm is 0.")
                return
            mean_orientation /= norm

            # Optional: broadcast the "averaged" ArUco marker TF for visualization
            self.broadcast_aruco_tf(mean_position, mean_orientation,
                                    child_frame='aruco_marker',
                                    parent_frame=self.pose_buffer[0].header.frame_id)

            # ----------------------------
            # 2) Lookup calibration_link & aruco_marker w.r.t. base_link
            # ----------------------------
            try:
                t_calib = self.tf_buffer.lookup_transform(
                    self.base_link_frame,
                    self.calibration_link_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except Exception as e:
                self.get_logger().error(f"Could not lookup transform base_link->calibration_link: {e}")
                return

            try:
                t_aruco = self.tf_buffer.lookup_transform(
                    self.base_link_frame,
                    'aruco_marker',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except Exception as e:
                self.get_logger().error(f"Could not lookup transform base_link->aruco_marker: {e}")
                return

            # ----------------------------
            # 3) Compute T_calib_aruco
            # ----------------------------
            T_base_calib = self.transform_to_matrix(t_calib.transform)
            T_base_aruco = self.transform_to_matrix(t_aruco.transform)
            T_calib_aruco = np.linalg.inv(T_base_calib) @ T_base_aruco

            # Extract translation
            diff_translation = T_calib_aruco[0:3, 3]

            # Extract quaternion
            diff_quat = quaternion_from_matrix(T_calib_aruco)

            # ----------------------------
            # 4) Convert to RPY
            # ----------------------------
            roll, pitch, yaw = euler_from_quaternion(diff_quat)

            # (Optional) Adjust RPY if needed:
            # For example, if your marker is physically rotated so that
            # the Z-axis is "sideways," you could do something like:
            # roll -= np.pi / 2
            # Or yaw += np.pi / 2, etc.

            # ----------------------------
            # 5) Convert back to quaternion
            # ----------------------------
            diff_quat_rpy = quaternion_from_euler(roll, pitch, yaw)

            # ----------------------------
            # 6) Save difference to YAML (only once)
            # ----------------------------
            self.save_calibration_to_yaml(diff_translation, diff_quat_rpy)
            # Mark calibration as done
            self.calibration_done = True

        except Exception as e:
            self.get_logger().error(f"Calibration processing failed: {e}")

    def broadcast_aruco_tf(self, position, orientation, child_frame, parent_frame):
        """
        Optionally broadcast the average ArUco marker TF for reference.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Broadcasting ArUco TF in {parent_frame}->{child_frame}")

    def filter_outliers_iqr(self, data):
        """
        Filters outliers using the IQR method (on positions).
        """
        q1 = np.percentile(data, 25, axis=0)
        q3 = np.percentile(data, 75, axis=0)
        iqr = q3 - q1
        lower_bound = q1 - self.IQR_MULTIPLIER * iqr
        upper_bound = q3 + self.IQR_MULTIPLIER * iqr

        valid = np.all((data >= lower_bound) & (data <= upper_bound), axis=1)
        return valid

    def transform_to_matrix(self, transform):
        """
        Convert a geometry_msgs Transform to a 4x4 NumPy matrix.
        """
        tx = transform.translation.x
        ty = transform.translation.y
        tz = transform.translation.z
        rx = transform.rotation.x
        ry = transform.rotation.y
        rz = transform.rotation.z
        rw = transform.rotation.w

        mat = quaternion_matrix([rx, ry, rz, rw])
        mat[0, 3] = tx
        mat[1, 3] = ty
        mat[2, 3] = tz
        return mat

    def save_calibration_to_yaml(self, translation, quaternion):
        """
        Saves the calibration difference (translation + orientation) to a YAML file.
        """
        out_file = os.path.join(self.ws_src_path, 'aruco_calibration.yaml')

        calibration_data = {
            'aruco_to_calibration_link': {
                'translation': {
                    'x': float(translation[0]),
                    'y': float(translation[1]),
                    'z': float(translation[2])
                },
                'rotation': {
                    'x': float(quaternion[0]),
                    'y': float(quaternion[1]),
                    'z': float(quaternion[2]),
                    'w': float(quaternion[3])
                }
            }
        }

        try:
            with open(out_file, 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=False)
            self.get_logger().info(f"calibration file saved to {out_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to write calibration file to {out_file}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
