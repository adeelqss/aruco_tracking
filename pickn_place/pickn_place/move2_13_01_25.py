#!/usr/bin/env python3
"""
Example of moving 'tool_link' to a TF-based pose using pymoveit2
with separate scaling for x, y, z positions and waiting for TF to arrive.

Usage:
  1) Launch your robot and MoveIt2 (plus any TF broadcaster for 'target_tf').
  2) ros2 run my_package ex_pose_tf_goal.py --ros-args \
       -p target_frame:=target_tf -p reference_frame:=world \
       -p movement_scaling_x:=0.95 -p movement_scaling_y:=0.90 -p movement_scaling_z:=1.0 \
       -p synchronous:=True
"""

from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener, LookupException
from pymoveit2 import MoveIt2, MoveIt2State


class TfMoveItExample(Node):
    def __init__(self):
        super().__init__("ex_pose_tf_goal")

        # Declare ROS parameters (override via --ros-args)
        self.declare_parameter("target_frame", "mount_link")
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("planner_id", "OMPL")
        self.declare_parameter("cartesian", True)
        self.declare_parameter("cartesian_max_step", 0.1)
        self.declare_parameter("cartesian_fraction_threshold", 0.0)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("cartesian_avoid_collisions", True)
        self.declare_parameter("velocity_scaling", 1.0)
        self.declare_parameter("acceleration_scaling", 1.0)

        # Individual scaling factors
        self.declare_parameter("movement_scaling_x", 1.0)  # Scale X
        self.declare_parameter("movement_scaling_y", 1.0)  # Scale Y
        self.declare_parameter("movement_scaling_z", 1.0)  # Scale Z

        self.declare_parameter("synchronous", True)

        # Read parameters
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.reference_frame = self.get_parameter("reference_frame").get_parameter_value().string_value
        self.planner_id = self.get_parameter("planner_id").get_parameter_value().string_value
        self.cartesian = self.get_parameter("cartesian").get_parameter_value().bool_value
        self.cartesian_max_step = self.get_parameter("cartesian_max_step").get_parameter_value().double_value
        self.cartesian_fraction_threshold = self.get_parameter("cartesian_fraction_threshold").get_parameter_value().double_value
        self.cartesian_jump_threshold = self.get_parameter("cartesian_jump_threshold").get_parameter_value().double_value
        self.cartesian_avoid_collisions = self.get_parameter("cartesian_avoid_collisions").get_parameter_value().bool_value
        self.velocity_scaling = self.get_parameter("velocity_scaling").get_parameter_value().double_value
        self.acceleration_scaling = self.get_parameter("acceleration_scaling").get_parameter_value().double_value

        # Separate scaling parameters for X/Y/Z
        self.movement_scaling_x = self.get_parameter("movement_scaling_x").get_parameter_value().double_value
        self.movement_scaling_y = self.get_parameter("movement_scaling_y").get_parameter_value().double_value
        self.movement_scaling_z = self.get_parameter("movement_scaling_z").get_parameter_value().double_value

        self.synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value

        # Create TF Buffer/Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create callback group for multi-threaded execution
        self.callback_group = ReentrantCallbackGroup()

        # Initialize MoveIt2 with your robot details
        self.moveit2 = MoveIt2(
            node=self,
            callback_group=self.callback_group,
            joint_names=[
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
            ],
            base_link_name="base_link",
            end_effector_name="tool_link",
            group_name="center_tool_link",  # SRDF group with tip link = tool_link
        )

        # Set desired planner
        self.moveit2.planner_id = self.planner_id

        # Velocity/acceleration scaling
        self.moveit2.max_velocity = self.velocity_scaling
        self.moveit2.max_acceleration = self.acceleration_scaling

        # Cartesian parameters (used if cartesian=True)
        self.moveit2.cartesian_jump_threshold = self.cartesian_jump_threshold
        self.moveit2.cartesian_avoid_collisions = self.cartesian_avoid_collisions

    def scale_position(self, position):
        """
        Scales the X, Y, and Z coordinates independently.
        position = [x, y, z]
        """
        x_scaled = position[0] * self.movement_scaling_x
        y_scaled = position[1] * self.movement_scaling_y
        z_scaled = position[2] * self.movement_scaling_z
        return [x_scaled, y_scaled, z_scaled]

    def run(self):
        """
        Main routine:
          1) Wait/spin to fill TF buffer
          2) Use can_transform() with a timeout
          3) Lookup TF from reference_frame to target_frame
          4) Convert to position + orientation
          5) Apply scaling ONLY to position
          6) Plan and execute
          7) If synchronous: wait_until_executed(); else: monitor asynchronously
        """

        # 1) Spin once or sleep to let TF data arrive
        self.create_rate(0.5).sleep()  # half-second sleep for buffer

        # 2) Check if we can transform within 2 seconds
        if not self.tf_buffer.can_transform(
            self.reference_frame,
            self.target_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=2.0)
        ):
            self.get_logger().warn(
                f"No transform from [{self.target_frame}] to [{self.reference_frame}] within 2s"
            )
            return

        # 3) Now that we verified availability, do the actual lookup
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                rclpy.time.Time(),  # "latest" transform
            )
        except LookupException as ex:
            self.get_logger().error(f"TF lookup failed even after can_transform: {ex}")
            return

        # 4) Extract position/orientation
        tx = transform_stamped.transform.translation.x
        ty = transform_stamped.transform.translation.y
        tz = transform_stamped.transform.translation.z
        rx = transform_stamped.transform.rotation.x
        ry = transform_stamped.transform.rotation.y
        rz = transform_stamped.transform.rotation.z
        rw = transform_stamped.transform.rotation.w
        original_position = [tx, ty, tz]
        original_quat_xyzw = [rx, ry, rz, rw]

        # 5) Apply scaling only to the position
        scaled_position = self.scale_position(original_position)
        # orientation is unchanged
        quat_xyzw = original_quat_xyzw

        self.get_logger().info(
            f"Moving tool_link from '{self.reference_frame}' toward '{self.target_frame}'\n"
            f"  Original position = {original_position}\n"
            f"  Scaled position   = {scaled_position}\n"
            f"  Orientation       = {quat_xyzw}\n"
            f"  cartesian         = {self.cartesian}\n"
            f"  movement_scaling_x={self.movement_scaling_x}, "
            f"movement_scaling_y={self.movement_scaling_y}, "
            f"movement_scaling_z={self.movement_scaling_z}\n"
            f"  synchronous       = {self.synchronous}\n"
        )

        # 6) Send goal to MoveIt2
        self.moveit2.move_to_pose(
            position=scaled_position,
            quat_xyzw=quat_xyzw,
            cartesian=self.cartesian,
            cartesian_max_step=self.cartesian_max_step,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold,
        )

        # 7) Synchronous or Asynchronous
        if self.synchronous:
            # Synchronous: wait until execution finishes or fails
            self.moveit2.wait_until_executed()
            state = self.moveit2.query_state()
            if state == MoveIt2State.IDLE:
                self.get_logger().info("Motion complete (synchronous).")
            else:
                self.get_logger().warn(f"Motion ended with state: {state}")
        else:
            # Asynchronous: do not block
            self.get_logger().info("Asynchronous execution started.")
            rate = self.create_rate(10)

            # Wait until the motion starts
            while self.moveit2.query_state() not in [
                MoveIt2State.EXECUTING,
                MoveIt2State.IDLE,
            ]:
                rate.sleep()

            future = self.moveit2.get_execution_future()
            while not future.done():
                self.get_logger().info(f"Current state: {self.moveit2.query_state()}")
                rate.sleep()

            # Once future is done, check result
            result = future.result()
            self.get_logger().info(
                f"Asynchronous motion done. Status: {result.status}, error_code: {result.result.error_code}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = TfMoveItExample()

    # Spin up the node in a multi-threaded executor
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Give TF and everything a moment to initialize
    node.create_rate(1.0).sleep()

    # Run the main routine
    node.run()

    # Cleanup
    rclpy.shutdown()
    executor_thread.join()
    node.destroy_node()


if __name__ == "__main__":
    main()
