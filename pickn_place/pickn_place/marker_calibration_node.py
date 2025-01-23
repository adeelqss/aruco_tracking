#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import yaml
import os

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_from_matrix
from rclpy.duration import Duration

# ------------------------------------------------------------------------------
# Helper: 4x4 transform <-> (translation, quaternion)
# ------------------------------------------------------------------------------

def pose_to_matrix(translation, quaternion):
    """
    Convert (t_x, t_y, t_z), (qx, qy, qz, qw) into a 4x4 homogeneous matrix.
    """
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
    """
    Convert 4x4 matrix into (translation, quaternion).
    """
    tx, ty, tz = M[0, 3], M[1, 3], M[2, 3]
    R = M[:3, :3]
    qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
    return (tx, ty, tz), (qx, qy, qz, qw)

def invert_transform(translation, quaternion):
    """
    Invert a transform described by (t, q).
    """
    T = pose_to_matrix(translation, quaternion)
    T_inv = np.linalg.inv(T)
    return matrix_to_pose(T_inv)

def multiply_transform(t1, q1, t2, q2):
    """
    Compose two transforms: (t1, q1) * (t2, q2).
    """
    M1 = pose_to_matrix(t1, q1)
    M2 = pose_to_matrix(t2, q2)
    Mout = M1 @ M2
    return matrix_to_pose(Mout)

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """
    Convert quaternion to 3x3 rotation matrix.
    """
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
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
# MarkerCalibrationNode
# ------------------------------------------------------------------------------
class MarkerCalibrationNode(Node):
    def __init__(self):
        super().__init__('marker_calibration_node')

        # Declare ROS 2 parameters
        self.declare_parameter('calibration_mode', False)
        self.declare_parameter('marker_data_file', 'data.yaml')
        self.declare_parameter('calibration_file', 'calibration.yaml')
        self.declare_parameter('visualize', True)

        # Retrieve parameter values
        self.calibration_mode = self.get_parameter('calibration_mode').get_parameter_value().bool_value
        self.marker_data_file = self.get_parameter('marker_data_file').get_parameter_value().string_value
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value

        # For simplicity, we only detect marker ID 39 in this example
        self.target_marker_id = 39

        # Attempt to load marker config from data.yaml
        self.load_marker_config()

        # TF2
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Calibration offset
        self.calibration_offset = None  # (trans, quat) in camera_color_optical_frame

        # If not in calibration mode, load offset from calibration_file
        if not self.calibration_mode:
            self.load_calibration_offset()

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

        # ArUco dictionary and parameters
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Create Subscribers
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        # For limiting calibration frequency
        self.last_calibration_time = 0.0

        self.get_logger().info("MarkerCalibrationNode is up and running!")


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
    # Load existing calibration offset
    # --------------------------------------------------------------------------
    def load_calibration_offset(self):
        if not os.path.exists(self.calibration_file):
            self.get_logger().warn(
                f"No calibration file found at {self.calibration_file}. Offset = None."
            )
            return
        with open(self.calibration_file, 'r') as f:
            data = yaml.safe_load(f)
        try:
            if data['calibration']['marker_id'] != self.target_marker_id:
                self.get_logger().warn("Calibration file marker_id differs from target_marker_id!")
            t = data['calibration']['translation']
            q = data['calibration']['quaternion']
            self.calibration_offset = (tuple(t), tuple(q))
            self.get_logger().info(
                f"Loaded calibration offset: {self.calibration_offset}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to read calibration data: {e}")
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
    # image_callback
    # --------------------------------------------------------------------------
    def image_callback(self, msg):
        # If no intrinsics yet, skip
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        # Convert to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ArUco detect
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        if ids is None or len(ids) == 0:
            self.display_frame(frame)
            return

        # Only marker 39
        idxs = np.where(ids == self.target_marker_id)[0]
        if len(idxs) < 1:
            self.display_frame(frame)
            return

        # We'll just handle the first instance
        c = [corners[i] for i in idxs]
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            c, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        tvec = tvecs[0][0]
        rvec = rvecs[0][0]

        # Convert rvec->rotation matrix->quaternion
        rmat, _ = cv2.Rodrigues(rvec)
        M = np.eye(4)
        M[:3, :3] = rmat
        quat = quaternion_from_matrix(M)

        # Marker pose in camera optical frame
        marker_t = (float(tvec[0]), float(tvec[1]), float(tvec[2]))
        marker_q = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))

        # Publish TF: camera_color_optical_frame -> aruco_marker
        # so we do NOT mix up frames with base_link
        self.publish_transform(
            parent_frame='camera_color_optical_frame',
            child_frame=self.marker_frame_name,
            translation=marker_t,
            quaternion=marker_q
        )

        # Calibration or Operation?
        if self.calibration_mode:
            self.calibrate_tool(marker_t, marker_q)
        else:
            self.publish_calibrated_tool(marker_t, marker_q)

        # Visualization
        self.display_frame(frame, rvec, tvec, c)


    # --------------------------------------------------------------------------
    # CALIBRATION LOGIC
    # --------------------------------------------------------------------------
    def calibrate_tool(self, marker_t, marker_q):
        """Compute offset so that marker -> tool transforms are known."""
        # Only do this every 5 seconds
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - self.last_calibration_time) < 5.0:
            return  # skip

        # We'll do a blocking wait for base_link->tool_link and base_link->camera_color_optical_frame
        now = rclpy.time.Time()
        if not self.tf_buffer.can_transform('base_link', 'tool_link', now, timeout=Duration(seconds=2.0)):
            self.get_logger().warn("Cannot transform base_link->tool_link after 2s.")
            return
        if not self.tf_buffer.can_transform('base_link', 'camera_color_optical_frame', now, timeout=Duration(seconds=2.0)):
            self.get_logger().warn("Cannot transform base_link->camera_color_optical_frame after 2s.")
            return

        try:
            # 1) base_link->tool_link
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

            # 2) base_link->camera_color_optical_frame
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

            # camera_color_optical_frame->base_link
            cam_bl_t, cam_bl_q = invert_transform(t_cam, q_cam)

            # camera_color_optical_frame->tool_link = (cam->base) * (base->tool)
            cam_tool_t, cam_tool_q = multiply_transform(cam_bl_t, cam_bl_q, t_tool, q_tool)

            # marker->tool_link = inverse(marker) * tool_link
            inv_marker_t, inv_marker_q = invert_transform(marker_t, marker_q)
            offset_t, offset_q = multiply_transform(inv_marker_t, inv_marker_q, cam_tool_t, cam_tool_q)

            # Save
            self.save_calibration_offset(offset_t, offset_q)
            self.get_logger().info("Calibration offset saved. You can stop or wait to recalc.")
            self.last_calibration_time = now_sec

        except Exception as e:
            self.get_logger().warn(f"Calibration transform error: {e}")


    def publish_calibrated_tool(self, marker_t, marker_q):
        """
        Use the loaded offset to publish base_link->tool_link_calibrated
        so you see a corrected tool frame in RViz.
        """
        if self.calibration_offset is None:
            return  # no offset known

        offset_t, offset_q = self.calibration_offset

        # camera_color_optical_frame->tool_link = marker->tool_offset
        # = (marker) * (offset)
        cam_tool_t, cam_tool_q = multiply_transform(marker_t, marker_q, offset_t, offset_q)

        # We want: base_link->tool_link_calibrated
        # So we need base_link->camera_color_optical_frame (again).
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
            # Then base_link->tool = (base_link->cam) * (cam->tool)
            bl_tool_t, bl_tool_q = multiply_transform(t_cam, q_cam, cam_tool_t, cam_tool_q)

            # Now publish base_link->tool_link_calibrated
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
        # Force them to be standard floats so we get a clean YAML
        offset_t = [float(x) for x in offset_t]
        offset_q = [float(x) for x in offset_q]

        data = {
            'calibration': {
                'marker_id': self.target_marker_id,
                'translation': offset_t,
                'quaternion': offset_q
            }
        }
        with open(self.calibration_file, 'w') as f:
            yaml.safe_dump(data, f)
        self.get_logger().info(f"Calibration offset saved to {self.calibration_file}")


    # --------------------------------------------------------------------------
    # Helper: Publish a TF
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
    # Optional: Show OpenCV window
    # --------------------------------------------------------------------------
    def display_frame(self, frame, rvec=None, tvec=None, corners=None):
        if self.visualize:
            if corners is not None and rvec is not None and tvec is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size * 0.5
                )
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

