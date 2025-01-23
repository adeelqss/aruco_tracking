#!/usr/bin/env python3

import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose
from tf_transformations import (
    quaternion_matrix,
    quaternion_from_matrix
)
from std_msgs.msg import Int32


class PerceptionTransformNode(Node):
    # Node Variables
    BUFFER_SIZE = 5  # Number of poses to accumulate before processing
    IQR_MULTIPLIER = 1.5  # Multiplier for IQR outlier filtering
    TF_TIMEOUT_SEC = 1  # Timeout for TF lookup in seconds
    FILTER_WINDOW_SIZE = 10  # Sliding window size for pose filtering
    TRANSLATION_THRESHOLD = 0.01  # Minimum positional change required (meters)
    ROTATION_THRESHOLD = 0.01  # Minimum rotational change required (radians)
    PUBLISH_INTERVAL_SEC = 1.0  # Interval to publish the same data if no significant change

    def __init__(self):
        super().__init__('perception_transform_node')

        # Load ROS parameters
        self.declare_parameter('aruco_pose_topic', '/pickn_place/aruco_pose')
        self.aruco_pose_topic = self.get_parameter('aruco_pose_topic').get_parameter_value().string_value

        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.declare_parameter('aruco_marker_frame', 'aruco_marker')
        self.aruco_marker_frame = self.get_parameter('aruco_marker_frame').get_parameter_value().string_value

        self.declare_parameter(
            'calibration_file',
            os.path.join(
                os.path.expanduser('~'),
                'barns_ws', 'src', 'pickn_place', 'share', 'aruco_calibration.yaml'
            )
        )
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value

        self.declare_parameter(
            'aruco_id_match_file',
            os.path.join(
                os.path.expanduser('~'),
                'barns_ws', 'src', 'pickn_place', 'share', 'arucoID_match.yaml'
            )
        )
        self.aruco_id_match_file = self.get_parameter('aruco_id_match_file').get_parameter_value().string_value

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timeout = Duration(seconds=self.TF_TIMEOUT_SEC)

        # Load calibration transform
        self.calib_translation, self.calib_quat = self.load_calibration_transform(self.calibration_file)
        if self.calib_translation is None or self.calib_quat is None:
            self.get_logger().warn("No valid calibration transform loaded. Using identity.")
            self.calib_translation = np.zeros(3)
            self.calib_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # Load ArUCo ID mapping
        self.aruco_id_map = self.load_aruco_id_mapping(self.aruco_id_match_file)

        # Pose buffer for accumulation
        self.pose_buffer = []

        # --- NEW: Publisher for the final filtered pose
        self.filtered_pose_pub = self.create_publisher(
            PoseStamped,
            '/pickn_place/filtered_aruco_pose',
            10
        )

        # Sliding window buffer for pose filtering
        self.sliding_window_buffer = []

        # Subscribers
        self.aruco_sub = self.create_subscription(
            PoseStamped,
            self.aruco_pose_topic,
            self.aruco_pose_callback,
            10
        )

        self.current_aruco_id = None
        self.aruco_id_sub = self.create_subscription(
            Int32,
            '/pickn_place/target_aruco_id',
            self.aruco_id_callback,
            10
        )

        # Timer for periodic publishing
        self.publish_timer = self.create_timer(self.PUBLISH_INTERVAL_SEC, self.publish_last_pose)

        # Last published pose
        self.last_published_pose = None

        # Log initial status
        self.get_logger().info(
            f"PerceptionTransformNode started.\n"
            f" - ArUCo pose topic: {self.aruco_pose_topic}\n"
            f" - Transform from Link6 to: {self.target_frame}\n"
            f" - aruco_marker_frame: {self.aruco_marker_frame}\n"
            f" - Loaded calibration file: {self.calibration_file}\n"
            f" - Loaded ID match file: {self.aruco_id_match_file}"
        )

    def load_aruco_id_mapping(self, yaml_file):
        if not os.path.exists(yaml_file):
            self.get_logger().warn(f"ID match file not found: {yaml_file}")
            return {}
        try:
            with open(yaml_file, 'r') as f:
                doc = yaml.safe_load(f)
            output_map = {}
            entries = doc.get('aruco_id', [])
            for entry in entries:
                mid = entry.get('id')
                mname = entry.get('name')
                if mid is not None and mname is not None:
                    output_map[mid] = mname
            self.get_logger().info(f"Loaded {len(output_map)} ID->Name entries from: {yaml_file}")
            return output_map
        except Exception as e:
            self.get_logger().error(f"Failed to parse ID match file: {e}")
            return {}

    def aruco_id_callback(self, msg: Int32):
        self.current_aruco_id = msg.data

    def load_calibration_transform(self, yaml_file):
        if not os.path.exists(yaml_file):
            self.get_logger().error(f"Calibration file not found: {yaml_file}")
            return None, None
        try:
            with open(yaml_file, 'r') as f:
                doc = yaml.safe_load(f)
            data = doc.get('aruco_to_calibration_link', {})
            trans = data.get('translation', {})
            rot = data.get('rotation', {})

            translation = np.array([
                trans.get('x', 0.0),
                trans.get('y', 0.0),
                trans.get('z', 0.0)
            ], dtype=float)
            quaternion = np.array([
                rot.get('x', 0.0),
                rot.get('y', 0.0),
                rot.get('z', 0.0),
                rot.get('w', 1.0)
            ], dtype=float)

            self.get_logger().info(
                f"Loaded calibration transform from YAML: "
                f"T_calib<-aruco = {translation}, {quaternion}"
            )
            return translation, quaternion
        except Exception as e:
            self.get_logger().error(f"Failed to parse calibration file: {e}")
            return None, None

    def aruco_pose_callback(self, msg: PoseStamped):
        self.pose_buffer.append(msg)
        if len(self.pose_buffer) >= self.BUFFER_SIZE:
            self.process_buffer()
            self.pose_buffer = []

    def process_buffer(self):
        """
        Accumulates several ArUCo poses, filters outliers using IQR,
        then computes an average pose and transforms it into the Link6 frame.
        Finally, broadcasts a transform and publishes a filtered pose.
        """
        # 1. Filter outliers based on IQR
        valid_indices = self.filter_outliers_iqr(self.pose_buffer)
        filtered_buffer = [self.pose_buffer[i] for i in range(len(self.pose_buffer)) if valid_indices[i]]
        if not filtered_buffer:
            self.get_logger().warning("No valid poses after filtering outliers.")
            return

        # 2. Compute the average pose from remaining data
        avg_pose = self.average_poses(filtered_buffer)
        camera_frame = avg_pose.header.frame_id

        # 3. Lookup transform from camera to Link6
        try:
            tf_cam_to_link6 = self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame,
                rclpy.time.Time(),
                self.tf_timeout
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {self.target_frame} <- {camera_frame}: {e}")
            return

        # 4. Transform average pose into Link6 frame
        pose_in_link6_pose = do_transform_pose(avg_pose.pose, tf_cam_to_link6)
        T_link6_marker = self.pose_to_matrix(pose_in_link6_pose)

        # 5. Apply calibration correction
        T_calib_aruco = self.make_matrix(self.calib_translation, self.calib_quat)
        T_link6_marker_corrected = T_link6_marker @ np.linalg.inv(T_calib_aruco)

        # --- REMOVED: Custom rotation swap ---

        # 6. Broadcast the transform (use the timestamp from the last input pose)
        last_stamp = filtered_buffer[-1].header.stamp
        self.broadcast_aruco_tf(T_link6_marker_corrected, last_stamp)

        # 7. Also publish the final marker pose as a topic
        final_pose_in_target_frame = self.matrix_to_pose(T_link6_marker_corrected)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.target_frame
        pose_stamped.header.stamp = last_stamp
        pose_stamped.pose = final_pose_in_target_frame

        # 7. Apply sliding window filter
        self.sliding_window_buffer.append(pose_stamped)
        if len(self.sliding_window_buffer) > self.FILTER_WINDOW_SIZE:
            self.sliding_window_buffer.pop(0)
        filtered_pose = self.average_poses(self.sliding_window_buffer)

        # 8. Check for significant change
        if self.is_significant_change(filtered_pose.pose):
            self.last_published_pose = filtered_pose
            self.filtered_pose_pub.publish(filtered_pose)

    def publish_last_pose(self):
        if self.last_published_pose:
            self.filtered_pose_pub.publish(self.last_published_pose)

    def is_significant_change(self, new_pose: Pose) -> bool:
        if not self.last_published_pose:
            return True

        last_pose = self.last_published_pose.pose
        translation_change = np.linalg.norm([
            new_pose.position.x - last_pose.position.x,
            new_pose.position.y - last_pose.position.y,
            new_pose.position.z - last_pose.position.z
        ])
        rotation_change = np.linalg.norm([
            new_pose.orientation.x - last_pose.orientation.x,
            new_pose.orientation.y - last_pose.orientation.y,
            new_pose.orientation.z - last_pose.orientation.z,
            new_pose.orientation.w - last_pose.orientation.w
        ])

        return (translation_change > self.TRANSLATION_THRESHOLD or
                rotation_change > self.ROTATION_THRESHOLD)

    def filter_outliers_iqr(self, poses):
        """
        Filters outlier positions using the Interquartile Range (IQR) method.
        Change the IQR multiplier if you need stricter or looser thresholds.
        """
        positions = np.array([
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in poses
        ])
        q1 = np.percentile(positions, 25, axis=0)
        q3 = np.percentile(positions, 75, axis=0)
        iqr = q3 - q1

        lower_bound = q1 - self.IQR_MULTIPLIER * iqr
        upper_bound = q3 + self.IQR_MULTIPLIER * iqr

        valid = np.all((positions >= lower_bound) & (positions <= upper_bound), axis=1)
        return valid

    def average_poses(self, pose_buffer):
        """
        Computes the average position and average quaternion (normalized)
        for a set of PoseStamped messages.
        """
        avg_position = np.mean([
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in pose_buffer
        ], axis=0)

        quaternions = np.array([
            [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
            for pose in pose_buffer
        ])
        avg_quaternion = np.mean(quaternions, axis=0)
        norm = np.linalg.norm(avg_quaternion)
        if norm < 1e-8:
            # fallback to identity if quaternions are degenerate
            avg_quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            avg_quaternion /= norm

        avg_pose = PoseStamped()
        # Use the last pose's header to keep timing close to real data
        avg_pose.header = pose_buffer[-1].header  
        avg_pose.pose.position.x = avg_position[0]
        avg_pose.pose.position.y = avg_position[1]
        avg_pose.pose.position.z = avg_position[2]
        avg_pose.pose.orientation.x = avg_quaternion[0]
        avg_pose.pose.orientation.y = avg_quaternion[1]
        avg_pose.pose.orientation.z = avg_quaternion[2]
        avg_pose.pose.orientation.w = avg_quaternion[3]
        return avg_pose

    def pose_to_matrix(self, pose: Pose) -> np.ndarray:
        px, py, pz = pose.position.x, pose.position.y, pose.position.z
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        mat = quaternion_matrix([qx, qy, qz, qw])
        mat[0, 3] = px
        mat[1, 3] = py
        mat[2, 3] = pz
        return mat

    def matrix_to_pose(self, mat: np.ndarray) -> Pose:
        """Helper to convert a 4x4 transform matrix into a geometry_msgs/Pose."""
        # Extract translation
        px = mat[0, 3]
        py = mat[1, 3]
        pz = mat[2, 3]
        # Extract rotation
        quat = quaternion_from_matrix(mat)

        pose = Pose()
        pose.position.x = float(px)
        pose.position.y = float(py)
        pose.position.z = float(pz)
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def make_matrix(self, translation: np.ndarray, quat: np.ndarray) -> np.ndarray:
        mat = quaternion_matrix(quat)
        mat[0, 3] = translation[0]
        mat[1, 3] = translation[1]
        mat[2, 3] = translation[2]
        return mat

    def broadcast_aruco_tf(self, T_link6_marker: np.ndarray, stamp):
        """
        Broadcasts the transform from Link6 to the ArUco marker frame,
        using the given timestamp.
        """
        tx = T_link6_marker[0, 3]
        ty = T_link6_marker[1, 3]
        tz = T_link6_marker[2, 3]
        quat = quaternion_from_matrix(T_link6_marker)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.target_frame

        child_name = self.aruco_marker_frame
        if self.current_aruco_id is not None:
            mapped_name = self.aruco_id_map.get(self.current_aruco_id)
            if mapped_name:
                child_name = mapped_name
            else:
                child_name = f"Aruco_{self.current_aruco_id}"
        t.child_frame_id = child_name

        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = float(tz)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
