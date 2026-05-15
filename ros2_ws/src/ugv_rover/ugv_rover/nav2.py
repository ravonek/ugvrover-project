#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ColorGoalNavigator(Node):
    def __init__(self) -> None:
        super().__init__("color_goal_navigator")

        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10
        )

        self.goal_done_pub = self.create_publisher(
            Bool,
            "/goal_done",
            10
        )

        self.cube_sent_done_pub = self.create_publisher(
            Bool,
            "/cube_sent_done",
            10
        )

        self.color_sub = self.create_subscription(
            String,
            "/colors",
            self.color_callback,
            10
        )

        self.initial_pose = (0.0, 1.5, 0.0)

        # Первая последовательность для любого цвета
        self.first_goal_sequence = [
            (0.6, 1.5, math.radians(0)),
            (0.6, 1.0, math.radians(-90)),
            (0.1, 0.9, math.radians(180)),
            (0.1, 0.1, math.radians(-90)),
            (0.1, 0.4, math.radians(90)),
        ]

        # Вторая последовательность для RED
        self.red_drop_sequence = [
            (0.1, 0.9, math.radians(90)),
            (1.13, 1.0, math.radians(0)),
            (1.13, 1.6, math.radians(90)),
            (1.13, 1.4, math.radians(-90)),
        ]

        # Вторая цель для GREEN
        self.green_drop_sequence = [
            (0.1, 0.9, math.radians(90)),
            (0.6, 1.0, math.radians(0)),
            (0.6, 0.4, math.radians(-90)),
            (1.2, 0.4, math.radians(0)),
            (1.1, 0.4, math.radians(180)),
        ]

        # Вторая цель для YELLOW
        self.yellow_drop_sequence = [
            (-2.0, 0.3, math.radians(0)),
        ]

        self.current_sequence_index = 0
        self.current_drop_index = 0

        self.initial_pose_sent = False
        self.goal_in_progress = False

        self.current_color = None
        self.current_drop_sequence = []

        # idle / first_sequence / drop_sequence
        self.stage = "idle"

        self.get_logger().info("ColorGoalNavigator started, waiting for /colors...")

    def publish_initial_pose_once(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)

        msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.02
        ]

        for _ in range(10):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(msg)
            time.sleep(0.2)

        self.initial_pose_sent = True

        self.get_logger().info(
            f"Initial pose published: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

    def publish_goal_done(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.goal_done_pub.publish(msg)
        self.get_logger().info(f"Published to /goal_done: {value}")

    def publish_cube_sent_done(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.cube_sent_done_pub.publish(msg)
        self.get_logger().info(f"Published to /cube_sent_done: {value}")

    def color_callback(self, msg: String) -> None:
        color = msg.data.strip().lower()
        self.get_logger().info(f"Received color: {color}")

        if self.goal_in_progress:
            self.get_logger().info("Goal already in progress, ignoring new color.")
            return

        if self.stage != "idle":
            self.get_logger().info(f"System is not idle. Current stage: {self.stage}")
            return

        if color == "red":
            self.current_drop_sequence = self.red_drop_sequence

        elif color == "green":
            self.current_drop_sequence = self.green_drop_sequence

        elif color == "yellow":
            self.current_drop_sequence = self.yellow_drop_sequence

        else:
            self.get_logger().info("Unknown color, ignoring.")
            return

        self.current_color = color
        self.stage = "first_sequence"
        self.current_sequence_index = 0
        self.current_drop_index = 0

        self.publish_goal_done(False)
        self.publish_cube_sent_done(False)

        if not self.initial_pose_sent:
            self.get_logger().info("Publishing initial pose before navigation...")
            self.publish_initial_pose_once(*self.initial_pose)
            time.sleep(4.0)

        self.send_next_first_goal()

    def send_next_first_goal(self) -> None:
        if self.current_sequence_index >= len(self.first_goal_sequence):
            self.get_logger().info("First goal sequence finished.")

            self.publish_goal_done(True)
            time.sleep(0.5)
            self.publish_goal_done(False)

            self.stage = "drop_sequence"
            self.goal_in_progress = False
            self.current_drop_index = 0

            self.send_next_drop_goal()
            return

        goal = self.first_goal_sequence[self.current_sequence_index]

        self.get_logger().info(
            f"Sending FIRST sequence goal {self.current_sequence_index + 1}/"
            f"{len(self.first_goal_sequence)}"
        )

        self.send_goal_async(*goal)

    def send_next_drop_goal(self) -> None:
        if self.current_drop_index >= len(self.current_drop_sequence):
            self.get_logger().info("Drop/send sequence finished.")

            self.publish_cube_sent_done(True)
            time.sleep(0.5)
            self.publish_cube_sent_done(False)

            self.reset_state()

            self.get_logger().info("Cycle finished. Waiting for next color.")
            return

        goal = self.current_drop_sequence[self.current_drop_index]

        self.get_logger().info(
            f"Sending DROP sequence goal {self.current_drop_index + 1}/"
            f"{len(self.current_drop_sequence)} for color: {self.current_color}"
        )

        self.send_goal_async(*goal)

    def send_goal_async(self, x: float, y: float, yaw: float) -> None:
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/navigate_to_pose action server not available")

            if self.stage == "first_sequence":
                self.publish_goal_done(False)
            elif self.stage == "drop_sequence":
                self.publish_cube_sent_done(False)

            self.reset_state()
            return

        self.goal_in_progress = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quaternion(yaw)

        self.get_logger().info(
            f"Sending goal: x={x:.2f}, y={y:.2f}, "
            f"yaw={math.degrees(yaw):.1f} deg, stage={self.stage}"
        )

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error("Failed to get goal handle")
            self.reset_state()
            return

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")

            if self.stage == "first_sequence":
                self.publish_goal_done(False)
            elif self.stage == "drop_sequence":
                self.publish_cube_sent_done(False)

            self.reset_state()
            return

        self.get_logger().info("Goal accepted, waiting for result...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future) -> None:
        result = future.result()

        if result is None:
            self.get_logger().error("No result returned")

            if self.stage == "first_sequence":
                self.publish_goal_done(False)
            elif self.stage == "drop_sequence":
                self.publish_cube_sent_done(False)

            self.reset_state()
            return

        status = result.status

        if status != 4:
            self.get_logger().warn(f"Goal finished with status: {status}")

            if self.stage == "first_sequence":
                self.publish_goal_done(False)
            elif self.stage == "drop_sequence":
                self.publish_cube_sent_done(False)

            self.reset_state()
            return

        if self.stage == "first_sequence":
            self.get_logger().info(
                f"FIRST sequence goal {self.current_sequence_index + 1} reached."
            )

            self.current_sequence_index += 1
            self.goal_in_progress = False

            time.sleep(0.3)
            self.send_next_first_goal()
            return

        if self.stage == "drop_sequence":
            self.get_logger().info(
                f"DROP sequence goal {self.current_drop_index + 1} reached "
                f"for color: {self.current_color}"
            )

            self.current_drop_index += 1
            self.goal_in_progress = False

            time.sleep(0.3)
            self.send_next_drop_goal()
            return

    def reset_state(self) -> None:
        self.goal_in_progress = False
        self.stage = "idle"
        self.current_color = None
        self.current_drop_sequence = []
        self.current_sequence_index = 0
        self.current_drop_index = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorGoalNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()