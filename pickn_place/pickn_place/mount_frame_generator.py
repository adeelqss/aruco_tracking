import os
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_matrix, quaternion_from_matrix
from ament_index_python.packages import get_package_share_directory

class TFDistanceCalculator(Node):
    """
    Modified node that:
      - Subscribes to /pickn_place/filtered_aruco_pose (PoseStamped).
      - Converts that pose to a transform matrix (parent_frame -> "input_pose").
      - Applies one or more offsets from a YAML file.
      - Broadcasts them as parent_frame -> child_frame transforms, 
        where parent_frame = msg.header.frame_id.
    """

    def __init__(self):
        super().__init__('tf_distance_calculator_mod')

        # -------------------------
        # User-configurable fields
        # -------------------------
        self.filtered_pose_topic = '/pickn_place/filtered_aruco_pose'

        # Retrieve the path to the mount_offsets.yaml in 'pickn_place' package
        share_dir = get_package_share_directory('pickn_place')
        self.yaml_file_path = os.path.join(share_dir, 'mount_offsets.yaml')

        # Create TF Broadcaster (no buffer/listener, since we no longer look up TF)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Dictionary to track the last broadcast data
        # (Key: (parent_frame, child_frame); Value: transform dict)
        self.last_broadcasts = {}

        # Load offsets from YAML
        self.yaml_data = self.load_yaml_data()

        # Subscribe to the final filtered ArUCo pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.filtered_pose_topic,
            self.pose_callback,
            10
        )

        # Timer to periodically re-broadcast the last known transforms
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Minimal logging
        self.get_logger().info(
            f"TFDistanceCalculator (modified) initialized:\n"
            f" - Subscribing to: {self.filtered_pose_topic}\n"
            f" - Offsets from: {self.yaml_file_path}"
        )

    def load_yaml_data(self):
        """Loads mount_offsets.yaml into a Python dictionary."""
        if not os.path.exists(self.yaml_file_path):
            raise FileNotFoundError(f"YAML file not found: {self.yaml_file_path}")

        try:
            with open(self.yaml_file_path, 'r') as file:
                data = yaml.safe_load(file) or {}
            self.get_logger().info(f"Loaded offsets from YAML with {len(data)} top-level entries.")
            return data
        except yaml.YAMLError as e:
            raise RuntimeError(f"Error parsing YAML file: {e}")

    def pose_callback(self, msg: PoseStamped):
        """
        Called whenever a new ArUCo pose arrives.
        
        1) Convert the input Pose into a matrix T_parent_input.
           (parent_frame = msg.header.frame_id)

        2) For each offset in the YAML, multiply T_parent_input by T_input_child
           (the offset) => T_parent_child.

        3) Broadcast parent_frame -> child_frame with T_parent_child.
        """
        parent_frame = msg.header.frame_id

        # 1) Convert the Pose itself into a 4x4 transform
        T_parent_input = self.pose_to_matrix(msg.pose)

        # 2) For each top-level key in the YAML, we have child frames
        for offset_key, offsets_dict in self.yaml_data.items():
            for child_frame, link_data in offsets_dict.items():
                # T_parent_child = T_parent_input * T_input_child
                T_input_child = self.offset_to_matrix(link_data)
                T_parent_child = T_parent_input @ T_input_child

                # Broadcast
                self.broadcast_transform(
                    T_parent_child,
                    stamp=msg.header.stamp,
                    parent_frame=parent_frame,
                    child_frame=child_frame
                )

    def pose_to_matrix(self, pose) -> np.ndarray:
        """
        Convert a geometry_msgs/Pose to a 4x4 transform matrix.
        Pose is relative to its header.frame_id.
        """
        tx = pose.position.x
        ty = pose.position.y
        tz = pose.position.z

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        mat = quaternion_matrix([qx, qy, qz, qw])
        mat[0, 3] = tx
        mat[1, 3] = ty
        mat[2, 3] = tz
        return mat

    def offset_to_matrix(self, link_data: dict) -> np.ndarray:
        """
        Convert YAML offset dict to a 4x4 transform matrix.
        Example link_data:
          {
            'translation': {'x': 0.1, 'y': 0.2, 'z': 0.3},
            'rotation':    {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
          }
        """
        tx = link_data['translation']['x']
        ty = link_data['translation']['y']
        tz = link_data['translation']['z']
        qx = link_data['rotation']['x']
        qy = link_data['rotation']['y']
        qz = link_data['rotation']['z']
        qw = link_data['rotation']['w']

        mat = quaternion_matrix([qx, qy, qz, qw])
        mat[0, 3] = tx
        mat[1, 3] = ty
        mat[2, 3] = tz
        return mat

    def matrix_to_transform_dict(self, mat: np.ndarray) -> dict:
        """
        Convert a 4x4 matrix into a dictionary with
          {'translation': (x,y,z), 'rotation': (x,y,z,w)}.
        """
        tx, ty, tz = mat[0, 3], mat[1, 3], mat[2, 3]
        qx, qy, qz, qw = quaternion_from_matrix(mat)
        return {
            'translation': (tx, ty, tz),
            'rotation': (qx, qy, qz, qw)
        }

    def broadcast_transform(self, T_parent_child: np.ndarray, stamp, parent_frame: str, child_frame: str):
        """
        Broadcasts a transform (parent_frame -> child_frame) using a 4x4 matrix T_parent_child.
        Only broadcasts if the data changed from last time.
        """
        # Convert matrix to a dictionary for easy comparison
        transform_data = self.matrix_to_transform_dict(T_parent_child)

        # Build a key for last_broadcasts
        key = (parent_frame, child_frame)

        # Check if it changed
        old_data = self.last_broadcasts.get(key)
        if old_data == transform_data:
            # no change => no need to broadcast
            return

        # Build TransformStamped
        transform_msg = TransformStamped()
        transform_msg.header.stamp = stamp
        transform_msg.header.frame_id = parent_frame
        transform_msg.child_frame_id = child_frame

        # Fill in translation
        tx, ty, tz = transform_data['translation']
        transform_msg.transform.translation.x = tx
        transform_msg.transform.translation.y = ty
        transform_msg.transform.translation.z = tz

        # Fill in rotation
        qx, qy, qz, qw = transform_data['rotation']
        transform_msg.transform.rotation.x = qx
        transform_msg.transform.rotation.y = qy
        transform_msg.transform.rotation.z = qz
        transform_msg.transform.rotation.w = qw

        # Broadcast
        self.tf_broadcaster.sendTransform(transform_msg)

        # Save new data
        self.last_broadcasts[key] = transform_data
        self.get_logger().info(
            f"Broadcasted TF: '{parent_frame}' -> '{child_frame}'"
        )

    def timer_callback(self):
        """Periodically re-broadcast the last known transforms."""
        for (parent_frame, child_frame), transform_data in self.last_broadcasts.items():
            # Build TransformStamped
            transform_msg = TransformStamped()
            transform_msg.header.stamp = self.get_clock().now().to_msg()
            transform_msg.header.frame_id = parent_frame
            transform_msg.child_frame_id = child_frame

            # Fill in translation
            tx, ty, tz = transform_data['translation']
            transform_msg.transform.translation.x = tx
            transform_msg.transform.translation.y = ty
            transform_msg.transform.translation.z = tz

            # Fill in rotation
            qx, qy, qz, qw = transform_data['rotation']
            transform_msg.transform.rotation.x = qx
            transform_msg.transform.rotation.y = qy
            transform_msg.transform.rotation.z = qz
            transform_msg.transform.rotation.w = qw

            # Broadcast
            self.tf_broadcaster.sendTransform(transform_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFDistanceCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
