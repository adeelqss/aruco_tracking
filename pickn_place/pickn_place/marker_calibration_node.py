#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import yaml
import os
import math
from collections import deque

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf_transformations import (
    quaternion_from_matrix,
    quaternion_slerp,  # <-- for orientation SLERP filtering
)
from rclpy.duration import Duration

# ------------------------------------------------------------------------------
# Helper: 4x4 transform <-> (translation, quaternion)
# ------------------------------------------------------------------------------
def pose_to_matrix(translation, quaternion):
    tx, ty, tz = translation
    qx, qy, qz, qw = quaternion
    M = np.eye(4)
    R = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    M[:3, :3] = R
    M[0, 3] = tx
    M[1, 3] = ty
    M[2, 3] = tz
    return M

def matrix_to_pose(M):
    tx, ty, tz = M[0, 3], M[1, 3], M[2, 3]
    R = M[:3, :3]
    qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
    return (tx, ty, tz), (qx, qy, qz, qw)

def invert_transform(translation, quaternion):
    T = pose_to_matrix(translation, quaternion)
    T_inv = np.linalg.inv(T)
    return matrix_to_pose(T_inv)

def multiply_transform(t1, q1, t2, q2):
    M1 = pose_to_matrix(t1, q1)
    M2 = pose_to_matrix(t2, q2)
    M_out = M1 @ M2
    return matrix_to_pose(M_out)

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    R = np.array([
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)]
    ])
    return R

def rotation_matrix_to_quaternion(R):
    """
    Convert 3x3 rotation matrix to (qx, qy, qz, qw).
    """
    import math
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2,1] - R[1,2]) * s
        qy = (R[0,2] - R[2,0]) * s
        qz = (R[1,0] - R[0,1]) * s
    else:
        if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            qw = (R[2,1] - R[1,2]) / s
            qx = 0.25 * s
            qy = (R[0,1] + R[1,0]) / s
            qz = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            qw = (R[0,2] - R[2,0]) / s
            qx = (R[0,1] + R[1,0]) / s
            qy = 0.25 * s
            qz = (R[1,2] + R[2,1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            qw = (R[1,0] - R[0,1]) / s
            qx = (R[0,2] + R[2,0]) / s
            qy = (R[1,2] + R[2,1]) / s
            qz = 0.25 * s
    return (qx, qy, qz, qw)


# ------------------------------------------------------------------------------
# MarkerCalibrationNode with Exponential + SLERP Filtering
# ------------------------------------------------------------------------------
class MarkerCalibrationNode(Node):
    def __init__(self):
        super().__init__('marker_calibration_node')

        # Declare ROS parameters
        self.declare_parameter('calibration_mode', False)
        self.declare_parameter('marker_data_file', 'data.yaml')
        self.declare_parameter('calibration_file', 'calibration.yaml')
        self.declare_parameter('visualize', True)
        # Filter alpha (0.0 -> no update, 1.0 -> always new)
        self.declare_parameter('filter_alpha', 0.2)

        # Retrieve param values
        self.calibration_mode = self.get_parameter('calibration_mode').value
        self.marker_data_file = self.get_parameter('marker_data_file').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.visualize = self.get_parameter('visualize').value
        self.filter_alpha = self.get_parameter('filter_alpha').value

        # We track only marker ID 39 in this example
        self.target_marker_id = 39

        # Load marker config
        self.load_marker_config()

        # TF2
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Calibration offset
        self.calibration_offset = None  # (t, q)

        # If not in calibration mode, load offset
        if not self.calibration_mode:
            self.load_calibration_offset()

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

        # ArUco dictionary + parameters
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # For throttling calibration to every 5s
        self.last_calibration_time = 0.0

        # Filtered pose storage
        self.filtered_t = None
        self.filtered_q = None

        # Create subscribers
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        self.get_logger().info("MarkerCalibrationNode with filtering is up and running!")


    # --------------------------------------------------------------------------
    # Load marker config from data.yaml
    # --------------------------------------------------------------------------
    def load_marker_config(self):
        if not os.path.exists(self.marker_data_file):
            self.get_logger().error(f"Marker data file not found: {self.marker_data_file}")
            return

        with open(self.marker_data_file, 'r') as f:
            config = yaml.safe_load(f)
        try:
            self.marker_size = float(config['markers'][self.target_marker_id]['size'])
            self.marker_frame_name = config['markers'][self.target_marker_id]['frame_name']
            self.get_logger().info(
                f"Loaded marker config: ID {self.target_marker_id}, size={self.marker_size}, "
                f"frame_name={self.marker_frame_name}"
            )
        except KeyError as e:
            self.get_logger().error(f"Missing marker config for ID {self.target_marker_id}: {e}")
            self.marker_size = 0.05
            self.marker_frame_name = f"aruco_{self.target_marker_id}"


    # --------------------------------------------------------------------------
    # Load existing calibration offset (operational mode)
    # --------------------------------------------------------------------------
    def load_calibration_offset(self):
        if not os.path.exists(self.calibration_file):
            self.get_logger().warn(f"No calibration file found at {self.calibration_file}. Offset=None.")
            return

        with open(self.calibration_file, 'r') as f:
            data = yaml.safe_load(f)
        try:
            if data['calibration']['marker_id'] != self.target_marker_id:
                self.get_logger().warn("Calibration file marker_id differs from target_marker_id!")
            t = data['calibration']['translation']
            q = data['calibration']['quaternion']
            self.calibration_offset = (tuple(t), tuple(q))
            self.get_logger().info(f"Loaded calibration offset: {self.calibration_offset}")
        except Exception as e:
            self.get_logger().error(f"Failed reading calibration data: {e}")
            self.calibration_offset = None


    # --------------------------------------------------------------------------
    # camera_info callback
    # --------------------------------------------------------------------------
    def camera_info_callback(self, msg):
        if self.camera_matrix is None and self.dist_coeffs is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera intrinsics acquired.")


    # --------------------------------------------------------------------------
    # image callback
    # --------------------------------------------------------------------------
    def image_callback(self, msg):
        # If intrinsics not ready, skip
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        if ids is None or len(ids) == 0:
            self.display_frame(frame)
            return

        idxs = np.where(ids == self.target_marker_id)[0]
        if len(idxs) < 1:
            self.display_frame(frame)
            return

        # We'll just handle the first instance
        c = [corners[i] for i in idxs]
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            c, self.marker_size, self.camera_matrix, self.dist_coeffs
        )
        tvec = tvecs[0][0]  # (x, y, z)
        rvec = rvecs[0][0]  # (rx, ry, rz)

        # Convert to quaternion
        rmat, _ = cv2.Rodrigues(rvec)
        M = np.eye(4)
        M[:3, :3] = rmat
        quat = quaternion_from_matrix(M)

        # Raw pose in camera optical frame
        raw_t = (float(tvec[0]), float(tvec[1]), float(tvec[2]))
        raw_q = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))

        # Apply filtering
        filt_t, filt_q = self.filter_pose(raw_t, raw_q)

        # Publish marker transform in camera optical frame
        self.publish_transform(
            parent_frame='camera_color_optical_frame',
            child_frame=self.marker_frame_name,
            translation=filt_t,
            quaternion=filt_q
        )

        # Calibration or operational
        if self.calibration_mode:
            self.calibrate_tool(filt_t, filt_q)
        else:
            self.publish_calibrated_tool(filt_t, filt_q)

        # Visualize
        self.display_frame(frame, rvec, tvec, c)


    # --------------------------------------------------------------------------
    # Pose FILTER: exponential for translation, SLERP for orientation
    # --------------------------------------------------------------------------
    def filter_pose(self, raw_t, raw_q):
        """
        Exponential filter for translation, SLERP for orientation.
        alpha = self.filter_alpha
        - If first time, just initialize.
        """
        alpha = self.filter_alpha

        if self.filtered_t is None:
            self.filtered_t = raw_t
            self.filtered_q = raw_q
            return raw_t, raw_q

        # Filter translation
        ft = (
            alpha*raw_t[0] + (1.0 - alpha)*self.filtered_t[0],
            alpha*raw_t[1] + (1.0 - alpha)*self.filtered_t[1],
            alpha*raw_t[2] + (1.0 - alpha)*self.filtered_t[2]
        )

        # SLERP for orientation
        # We interpret quaternion_slerp(q1, q2, 0.0->1.0)
        # so we want "previous->raw" with fraction=alpha
        fq = quaternion_slerp(self.filtered_q, raw_q, alpha)

        self.filtered_t = ft
        self.filtered_q = fq
        return ft, fq


    # --------------------------------------------------------------------------
    # Calibration routine
    # --------------------------------------------------------------------------
    def calibrate_tool(self, marker_t, marker_q):
        """
        Every 5s, tries to compute offset from marker->tool_link in camera frame.
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - self.last_calibration_time) < 5.0:
            return

        now = rclpy.time.Time()

        # Need base_link->tool_link and base_link->camera_color_optical_frame
        if not self.tf_buffer.can_transform('base_link', 'tool_link', now, timeout=Duration(seconds=2.0)):
            self.get_logger().warn("Cannot transform base_link->tool_link after 2s.")
            return
        if not self.tf_buffer.can_transform('base_link', 'camera_color_optical_frame', now, timeout=Duration(seconds=2.0)):
            self.get_logger().warn("Cannot transform base_link->camera_color_optical_frame after 2s.")
            return

        try:
            bl_tool = self.tf_buffer.lookup_transform('base_link', 'tool_link', now)
            t_tool = (
                bl_tool.transform.translation.x,
                bl_tool.transform.translation.y,
                bl_tool.transform.translation.z
            )
            q_tool = (
                bl_tool.transform.rotation.x,
                bl_tool.transform.rotation.y,
                bl_tool.transform.rotation.z,
                bl_tool.transform.rotation.w
            )

            bl_cam = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', now)
            t_cam = (
                bl_cam.transform.translation.x,
                bl_cam.transform.translation.y,
                bl_cam.transform.translation.z
            )
            q_cam = (
                bl_cam.transform.rotation.x,
                bl_cam.transform.rotation.y,
                bl_cam.transform.rotation.z,
                bl_cam.transform.rotation.w
            )

            # camera->base
            cam_bl_t, cam_bl_q = invert_transform(t_cam, q_cam)

            # camera->tool = (camera->base) * (base->tool)
            cam_tool_t, cam_tool_q = multiply_transform(cam_bl_t, cam_bl_q, t_tool, q_tool)

            # marker->tool = inverse(marker) * tool
            inv_marker_t, inv_marker_q = invert_transform(marker_t, marker_q)
            offset_t, offset_q = multiply_transform(inv_marker_t, inv_marker_q, cam_tool_t, cam_tool_q)

            self.save_calibration_offset(offset_t, offset_q)
            self.get_logger().info("Calibration offset saved. You can stop or let it recalc.")
            self.last_calibration_time = now_sec

        except Exception as e:
            self.get_logger().warn(f"Calibrate tool error: {e}")


    # --------------------------------------------------------------------------
    # Publish calibrated tool
    # --------------------------------------------------------------------------
    def publish_calibrated_tool(self, marker_t, marker_q):
        if self.calibration_offset is None:
            return
        offset_t, offset_q = self.calibration_offset

        # camera->tool_calib = (marker) * (offset)
        cam_tool_t, cam_tool_q = multiply_transform(marker_t, marker_q, offset_t, offset_q)

        now = rclpy.time.Time()
        if not self.tf_buffer.can_transform('base_link', 'camera_color_optical_frame', now):
            return
        try:
            bl_cam = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', now)
            t_cam = (
                bl_cam.transform.translation.x,
                bl_cam.transform.translation.y,
                bl_cam.transform.translation.z
            )
            q_cam = (
                bl_cam.transform.rotation.x,
                bl_cam.transform.rotation.y,
                bl_cam.transform.rotation.z,
                bl_cam.transform.rotation.w
            )
            # base_link->tool_calib = (base_link->cam) * (cam->tool_calib)
            bl_tool_t, bl_tool_q = multiply_transform(t_cam, q_cam, cam_tool_t, cam_tool_q)

            self.publish_transform(
                parent_frame='base_link',
                child_frame='tool_link_calibrated',
                translation=bl_tool_t,
                quaternion=bl_tool_q
            )
        except Exception as e:
            self.get_logger().warn(f"publish_calibrated_tool error: {e}")


    # --------------------------------------------------------------------------
    # Save calibration offset
    # --------------------------------------------------------------------------
    def save_calibration_offset(self, offset_t, offset_q):
        # convert to floats
        ot = [float(x) for x in offset_t]
        oq = [float(x) for x in offset_q]

        data = {
            'calibration': {
                'marker_id': self.target_marker_id,
                'translation': ot,
                'quaternion': oq
            }
        }
        with open(self.calibration_file, 'w') as f:
            yaml.safe_dump(data, f)
        self.get_logger().info(f"Calibration offset saved to {self.calibration_file}")


    # --------------------------------------------------------------------------
    # Publish transform
    # --------------------------------------------------------------------------
    def publish_transform(self, parent_frame, child_frame, translation, quaternion):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(t)


    # --------------------------------------------------------------------------
    # (Optional) Display in OpenCV
    # --------------------------------------------------------------------------
    def display_frame(self, frame, rvec=None, tvec=None, corners=None):
        if self.visualize:
            if corners is not None and rvec is not None and tvec is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size * 0.5)
            cv2.imshow("Aruco Detection", frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down on Ctrl+C...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

