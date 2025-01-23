import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from transformations import quaternion_from_matrix
from std_msgs.msg import Int32
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from time import time

class ArucoPerceptionNode(Node):
    def __init__(self):
        super().__init__('aruco_perception_node')

        # ---------------------------
        # 1. Declare ROS parameters
        # ---------------------------
        self.declare_parameter('target_marker_id', 18                            )
        self.TARGET_MARKER_ID = self.get_parameter('target_marker_id').get_parameter_value().integer_value
        self.last_logged_id = None

        self.declare_parameter('visualize', True)
        self.VISUALIZE = self.get_parameter('visualize').get_parameter_value().bool_value

        # The threshold logic now includes the possibility of 0 disabling depth usage
        self.declare_parameter('depth_dist_threshold_mm', 200.0)
        self.DEPTH_DIST_THRESHOLD_MM = self.get_parameter('depth_dist_threshold_mm').get_parameter_value().double_value

        # -----------------------------------
        # 2. Basic settings and placeholders
        # -----------------------------------
        self.REPROJECTION_ERROR_THRESHOLD = 1.5

        self.last_log_time = 0
        self.LOG_INTERVAL = 5

        self.bridge = CvBridge()

        # ArUco detection parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshWinSizeMin = 5
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 4
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_solver = cv2.SOLVEPNP_ITERATIVE

        # Color camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_width = None
        self.image_height = None
        self.camera_info_received = False
        self.image_received = False

        # ---------------------------
        # 3. Depth camera placeholders
        # ---------------------------
        self.depth_camera_matrix = None
        self.depth_dist_coeffs = None
        self.depth_width = None
        self.depth_height = None
        self.depth_info_received = False
        self.depth_image_received = False
        self.latest_depth_image = None 

        # Load marker sizes
        self.CONFIG_FILE = os.path.join(
            get_package_share_directory('pickn_place'),
            'aruco_config.yaml'
        )
        try:
            self.marker_sizes = self.load_marker_sizes(self.CONFIG_FILE)
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            self.marker_sizes = {}

        # -------------------------------------------
        # 4. Create subscriptions for color + depth
        # -------------------------------------------

        # Orbbec color
        self.CAMERA_FRAME = 'camera_depth_optical_frame'  # Update if Orbbec uses a different frame
        self.IMAGE_TOPIC = '/camera/color/image_raw'
        self.CAMERA_INFO_TOPIC = '/camera/color/camera_info'
        
        # Orbbec depth (raw)
        self.depth_info_topic = '/camera/depth/camera_info'
        self.depth_image_topic = '/camera/depth/image_raw'

        # Create the subscriptions
        self.create_subscription(CameraInfo, self.CAMERA_INFO_TOPIC, self.camera_info_callback, 10)
        self.create_subscription(Image, self.IMAGE_TOPIC, self.image_callback, 10)

        self.create_subscription(CameraInfo, self.depth_info_topic, self.depth_info_callback, 10)
        self.create_subscription(Image, self.depth_image_topic, self.depth_callback, 10)

        # -----------------------------------
        # 5. Pose publisher + checks + params
        # -----------------------------------
        self.pose_pub = self.create_publisher(PoseStamped, '/pickn_place/aruco_pose', 10)
        
        # NEW: Publisher for the detected target ArUco ID
        self.aruco_id_pub = self.create_publisher(Int32, '/pickn_place/target_aruco_id', 10)

        self.create_timer(5.0, self.check_input_topics)
        self.add_on_set_parameters_callback(self.parameter_callback)

    # ------------------------------------------------
    # 6. Load marker sizes from external YAML config
    # ------------------------------------------------
    def load_marker_sizes(self, config_path):
        if not os.path.exists(config_path):
            self.throttled_log(f"Configuration file not found: {config_path}.", "warn")
            return {}

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            marker_sizes = {}
            for entry in config.get('aruco_markers', []):
                if 'id_range' in entry and 'size_mm' in entry:
                    id_range = range(entry['id_range'][0], entry['id_range'][1] + 1)
                    for marker_id in id_range:
                        size_m = entry['size_mm'] / 1000.0
                        marker_sizes[marker_id] = size_m
            self.throttled_log("Loaded marker sizes from configuration.", "info")
            return marker_sizes

    # ----------------------------------
    # 7. Throttled log helper
    # ----------------------------------
    def throttled_log(self, message, level="info"):
        current_time = time()
        if current_time - self.last_log_time >= self.LOG_INTERVAL:
            if level == "info":
                self.get_logger().info(message)
            elif level == "warn":
                self.get_logger().warn(message)
            elif level == "error":
                self.get_logger().error(message)
            self.last_log_time = current_time

    def get_marker_size(self, marker_id):
        size = self.marker_sizes.get(marker_id)
        if size is None:
            self.get_logger().error(f"Marker ID {marker_id} not found in configuration file.")
            raise ValueError(f"Marker ID {marker_id} size is not defined.")
        return size

    # ------------------------------------
    # 8. Color camera info callback
    # ------------------------------------
    def camera_info_callback(self, msg):
        # Only set if not already set
        if self.camera_matrix is None and self.dist_coeffs is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.image_width = msg.width
            self.image_height = msg.height
            self.camera_info_received = True
            self.throttled_log("Camera intrinsics and resolution set (color).", "info")

    # -------------------------------------
    # 9. Depth camera info callback
    # -------------------------------------
    def depth_info_callback(self, msg):
        if self.depth_camera_matrix is None and self.depth_dist_coeffs is None:
            self.depth_camera_matrix = np.array(msg.k).reshape(3, 3)
            self.depth_dist_coeffs = np.array(msg.d)
            self.depth_width = msg.width
            self.depth_height = msg.height
            self.depth_info_received = True
            self.throttled_log("Depth intrinsics and resolution set (raw).", "info")

    # -------------------------------------------
    # 10. Depth image callback (raw depth)
    # -------------------------------------------
    def depth_callback(self, msg):
        try:
            # Convert depth image (likely uint16) to a NumPy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = depth_image
            self.depth_image_received = True
        except cv2.error as e:
            self.throttled_log(f"Depth callback error: {e}", "error")

    # ------------------------------------
    # 11. Check if input topics are active
    # ------------------------------------
    def check_input_topics(self):
        if not self.camera_info_received:
            self.throttled_log("No color camera info received. Ensure /camera/color/camera_info is publishing.", "warn")
        if not self.image_received:
            self.throttled_log("No color image received. Ensure /camera/color/image_raw is publishing.", "warn")
        if not self.depth_info_received:
            self.throttled_log("No depth camera info received. Ensure /camera/depth/camera_info is publishing.", "warn")
        if not self.depth_image_received:
            self.throttled_log("No depth image received. Ensure /camera/depth/image_raw is publishing.", "warn")

    # ----------------------
    # 12. Optional cropping
    # ----------------------
    def crop_center(self, frame):
        if self.image_width is None or self.image_height is None:
            return frame  # If resolution is not set, skip cropping

        h, w, _ = frame.shape
        crop_x = min(self.image_width, w)
        crop_y = min(self.image_height, h)

        start_x = (w - crop_x) // 2
        start_y = (h - crop_y) // 2

        return frame[start_y:start_y + crop_y, start_x:start_x + crop_x]

    # ------------------------------------
    # 13. Color image callback
    # ------------------------------------
    def image_callback(self, msg):
        self.image_received = True
        # Wait for color camera intrinsics
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.throttled_log("Waiting for color camera intrinsics...", "warn")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = self.crop_center(frame)

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                corners_2d = corners[i][0]  # shape: (4, 2)
                try:
                    marker_size = self.get_marker_size(marker_id)
                except ValueError:
                    # Marker not in config file, skip
                    continue

                center_2d = np.mean(corners_2d, axis=0).astype(int)  # (u, v)

                # ----------------------------------
                # A) SolvePnP for orientation + dist
                # ----------------------------------
                obj_points = np.array([
                    [-marker_size / 2,  marker_size / 2, 0],
                    [ marker_size / 2,  marker_size / 2, 0],
                    [ marker_size / 2, -marker_size / 2, 0],
                    [-marker_size / 2, -marker_size / 2, 0]
                ], dtype=np.float32)

                success, rvec, tvec, inliers = cv2.solvePnPRansac(
                    obj_points,
                    corners_2d,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=self.aruco_solver
                )

                if not success or (inliers is not None and len(inliers) < self.REPROJECTION_ERROR_THRESHOLD):
                    continue

                # By default, use PnP distance
                final_tvec = tvec.copy()  # (3,1)
                distance_source = "PnP"
                distance_mm = np.linalg.norm(tvec) * 1000.0  # mm

                # ------------------------------------------------
                # B) Attempt to get a valid depth-based distance
                #    Only if threshold > 0 and depth is valid
                # ------------------------------------------------
                if (self.DEPTH_DIST_THRESHOLD_MM != 0.0
                    and self.latest_depth_image is not None
                    and 0 <= center_2d[0] < self.depth_width
                    and 0 <= center_2d[1] < self.depth_height):

                    raw_depth = self.latest_depth_image[center_2d[1], center_2d[0]]  # likely uint16
                    if raw_depth > 0:  # valid, non-zero depth reading
                        depth_m = float(raw_depth) / 1000.0
                        depth_dist_mm = depth_m * 1000.0

                        # If PnP is over threshold, use depth
                        if distance_mm > self.DEPTH_DIST_THRESHOLD_MM:
                            fx = self.depth_camera_matrix[0, 0]
                            fy = self.depth_camera_matrix[1, 1]
                            cx = self.depth_camera_matrix[0, 2]
                            cy = self.depth_camera_matrix[1, 2]

                            (u, v) = (center_2d[0], center_2d[1])
                            X = (u - cx) * depth_m / fx
                            Y = (v - cy) * depth_m / fy
                            Z = depth_m

                            final_tvec = np.array([[X], [Y], [Z]], dtype=np.float32)
                            distance_source = "Depth"
                            distance_mm = depth_dist_mm

                # -------------------------------------
                # C) Display only the chosen distance
                # -------------------------------------
                if self.VISUALIZE:
                    if distance_source == "Depth":
                        text = f"Depth Dist: {distance_mm:.2f} mm"
                        color = (0, 255, 0)  # Green
                    else:
                        text = f"PnP Dist: {distance_mm:.2f} mm"
                        color = (0, 255, 0)  # Green

                    cv2.putText(
                        frame,
                        text,
                        (center_2d[0], center_2d[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2
                    )

                    # Draw the detected ArUco marker border in the frame
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids[i])

                # If the marker is our target, also draw axes + publish pose
                if marker_id == self.TARGET_MARKER_ID:
                    # Publish the target ID (e.g. 42) on its own topic
                    self.aruco_id_pub.publish(Int32(data=int(marker_id)))

                    if self.last_logged_id != self.TARGET_MARKER_ID:
                        self.get_logger().info(
                            f"[ID {marker_id}] Using {distance_source} distance = {distance_mm:.2f} mm"
                        )
                        self.last_logged_id = self.TARGET_MARKER_ID

                    # Orientation from SolvePnP
                    rmat, _ = cv2.Rodrigues(rvec)
                    transform_matrix = np.eye(4)
                    transform_matrix[:3, :3] = rmat
                    quat = quaternion_from_matrix(transform_matrix)

                    # Optionally draw 3D axes in the image (axis length = marker_size/2)
                    if self.VISUALIZE:
                        cv2.drawFrameAxes(
                            frame,
                            self.camera_matrix,
                            self.dist_coeffs,
                            rvec,
                            final_tvec,
                            marker_size / 2
                        )

                    # Build a PoseStamped using final_tvec
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = self.get_clock().now().to_msg()
                    pose_stamped.header.frame_id = self.CAMERA_FRAME

                    pose_stamped.pose.position.x = float(final_tvec[0])
                    pose_stamped.pose.position.y = float(final_tvec[1])
                    pose_stamped.pose.position.z = float(final_tvec[2])

                    # Convert Rodrigues -> quaternion
                    # quaternion_from_matrix returns [x, y, z, w]
                    pose_stamped.pose.orientation.z = -float(quat[0])
                    pose_stamped.pose.orientation.y = float(quat[1])
                    pose_stamped.pose.orientation.x = -float(quat[2])
                    pose_stamped.pose.orientation.w = float(quat[3])

                    # Publish the final pose
                    self.pose_pub.publish(pose_stamped)

        # ---------------
        # Visualization
        # ---------------
        if self.VISUALIZE:
            try:
                cv2.imshow("Aruco Detection", frame)
                cv2.waitKey(1)  # Ensure the window is updated
            except cv2.error:
                pass

    # ------------------------------------
    # 14. Parameter update callback
    # ------------------------------------
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'target_marker_id':
                if param.type_ in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                    new_id = int(param.value)
                    self.TARGET_MARKER_ID = new_id
                    self.last_logged_id = None
                    marker_size = self.get_marker_size(new_id)
                    self.get_logger().info(
                        f"Overlaying 6D axes for target ID: {new_id}, Size: {marker_size:.4f}"
                    )
            elif param.name == 'visualize':
                if param.type_ == Parameter.Type.BOOL:
                    self.VISUALIZE = param.value
            elif param.name == 'depth_dist_threshold_mm':
                if param.type_ in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                    self.DEPTH_DIST_THRESHOLD_MM = float(param.value)
                    self.get_logger().info(
                        f"Depth distance threshold updated to {self.DEPTH_DIST_THRESHOLD_MM} mm"
                    )

        return SetParametersResult(successful=True)

    # ----------------------
    # 15. Cleanup
    # ----------------------
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

# 16. Main entry point
def main(args=None):
    rclpy.init(args=args)
    node = ArucoPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
