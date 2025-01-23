import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException
import yaml
import os

class TFOffsetCapture(Node):
    def __init__(self):
        super().__init__('tf_offset_capture')

        # Hardcoded path to match the other node's save location
        self.ws_src_path = os.path.expanduser('~/barns_ws/src/pickn_place/share')

        # ---- Load the arucoID_match.yaml file ----
        self.aruco_dict_file = os.path.join(self.ws_src_path, 'arucoID_match.yaml')
        if not os.path.exists(self.aruco_dict_file):
            self.get_logger().error(
                f"Cannot find 'arucoID_match.yaml' at {self.aruco_dict_file}"
            )
            raise FileNotFoundError(
                f"'arucoID_match.yaml' not found in {self.aruco_dict_file}"
            )

        with open(self.aruco_dict_file, 'r') as f:
            self.aruco_id_data = yaml.safe_load(f) or {}

        # Prompt the user for either numeric ID or a tool name
        self.origin_frame = self.ask_user_for_origin_frame()

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to capture the transforms once
        self.timer = self.create_timer(1.0, self.capture_transforms)

        self.get_logger().info(
            f"Looking for transforms from '{self.origin_frame}' "
            f"to 'tool_link' and 'Link6'"
        )

    def ask_user_for_origin_frame(self) -> str:
        """
        Prompt the user for an ArUco ID (numeric) or a tool name (string).
          - If numeric, look up the matching name in arucoID_match.yaml.
            If found, return the matched name. If not, say “no match on ID” and ask again.
          - If name, check if it exists in the YAML. If found, return that exact name.
            If not, say “no match on name” and ask again.
        """
        while True:
            user_input = input("Enter an ArUco ID (numeric) or a tool name to calibrate: ").strip()

            # Check if user_input is purely numeric (aruco ID)
            if user_input.isdigit():
                tool_name = self.find_name_by_id(int(user_input))
                if tool_name:
                    return tool_name
                else:
                    print(f"No match on ID {user_input}. Please try again.")
                    continue
            else:
                # Otherwise, treat it as a name. Check if it exists in the YAML.
                if self.name_exists_in_yaml(user_input):
                    return user_input  # Use the exact name as the frame
                else:
                    print(f"No match on name '{user_input}'. Please try again.")

    def find_name_by_id(self, numeric_id: int):
        """
        Given a numeric ArUco ID, return the corresponding 'name' from the YAML, or None if not found.
        """
        aruco_list = self.aruco_id_data.get('aruco_id', [])
        for entry in aruco_list:
            if entry.get('id') == numeric_id:
                return entry.get('name')
        return None

    def name_exists_in_yaml(self, name: str) -> bool:
        """
        Check if `name` appears in the list of 'name' fields within arucoID_match.yaml.
        Returns True if found, False otherwise.
        """
        aruco_list = self.aruco_id_data.get('aruco_id', [])
        for entry in aruco_list:
            if entry.get('name') == name:
                return True
        return False

    def capture_transforms(self):
        try:
            # 1) Lookup the transform (origin -> tool_link)
            transform_mount = self.tf_buffer.lookup_transform(
                self.origin_frame,
                'tool_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )

            # Extract translation/rotation
            mount_translation = transform_mount.transform.translation
            mount_rotation = transform_mount.transform.rotation

            # 2) Lookup the transform (origin -> Link6)
            transform_approach = self.tf_buffer.lookup_transform(
                self.origin_frame,
                'Link6',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )

            approach_translation = transform_approach.transform.translation
            approach_rotation = transform_approach.transform.rotation

            # Prepare the data for a single offset block
            offset_data = {
                'mount_link': {
                    'translation': {
                        'x': mount_translation.x,
                        'y': mount_translation.y,
                        'z': mount_translation.z,
                    },
                    'rotation': {
                        'x': mount_rotation.x,
                        'y': mount_rotation.y,
                        'z': mount_rotation.z,
                        'w': mount_rotation.w,
                    },
                },
                'approach_link': {
                    'translation': {
                        'x': approach_translation.x,
                        'y': approach_translation.y,
                        'z': approach_translation.z,
                    },
                    'rotation': {
                        'x': approach_rotation.x,
                        'y': approach_rotation.y,
                        'z': approach_rotation.z,
                        'w': approach_rotation.w,
                    },
                },
            }

            # ---- Key in mount_offsets.yaml: e.g. "portafilter_offset" ----
            yaml_file = os.path.join(self.ws_src_path, "mount_offsets.yaml")

            # Load existing data (if any)
            existing_data = {}
            if os.path.exists(yaml_file):
                try:
                    with open(yaml_file, 'r') as file:
                        existing_data = yaml.safe_load(file) or {}
                except yaml.YAMLError as e:
                    self.get_logger().warn(f"Could not parse existing YAML: {e}")
                    existing_data = {}

            # The new key name will be "<origin_frame>_offset"
            # e.g. if origin_frame = "portafilter", key = "portafilter_offset"
            offset_key = f"{self.origin_frame}_offset"

            # Overwrite or create new entry
            existing_data[offset_key] = offset_data

            # Save updated data
            with open(yaml_file, 'w') as file:
                yaml.dump(existing_data, file)

            self.get_logger().info(
                f"Offsets saved to {yaml_file} under key '{offset_key}'"
            )

            # Stop the timer after successful capture
            self.timer.cancel()

        except LookupException as e:
            self.get_logger().warn(f"Transform not found: {str(e)}")
        except Exception as e:
            self.get_logger().warn(f"Failed to lookup transforms: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    node = TFOffsetCapture()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
