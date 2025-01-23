import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException
import yaml
import os

class TFOffsetCapture(Node):
    def __init__(self):
        super().__init__('tf_offset_capture')

        # Parameters for frame names
        self.declare_parameter('origin_frame', 'portafilter')
        # You could still declare parameters for the two target frames if needed,
        # but for this example, we'll hardcode them in the code for clarity.

        # Get the origin frame from parameter
        self.origin_frame = self.get_parameter('origin_frame').get_parameter_value().string_value

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to capture the transforms once
        self.timer = self.create_timer(1.0, self.capture_transforms)

        self.get_logger().info(f"Looking for transforms from '{self.origin_frame}' to 'tool_link' and 'Link6'")

    def capture_transforms(self):
        try:
            # 1) Lookup the transform for "mount_link" (origin -> tool_link)
            transform_mount = self.tf_buffer.lookup_transform(
                self.origin_frame,
                'tool_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )

            # Extract translation and rotation for mount_link
            mount_translation = transform_mount.transform.translation
            mount_rotation = transform_mount.transform.rotation

            # 2) Lookup the transform for "approach_link" (origin -> Link6)
            transform_approach = self.tf_buffer.lookup_transform(
                self.origin_frame,
                'Link6',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )

            # Extract translation and rotation for approach_link
            approach_translation = transform_approach.transform.translation
            approach_rotation = transform_approach.transform.rotation

            # Prepare the data as a dictionary with two entries
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

            # Save the two sets of transforms in one YAML file
            yaml_file = os.path.join(
                os.getcwd(),
                f"{self.origin_frame}_offsets.yaml"
            )

            with open(yaml_file, 'w') as file:
                yaml.dump(offset_data, file)

            self.get_logger().info(f"Offsets saved to {yaml_file}")

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
