#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Float64MultiArray


class PickPlaceIsaacSim(Node):
    def __init__(self):
        super().__init__("pick_place_isaacsim")

        self.joint_pub = self.create_publisher(JointState, "/joint_command", 10)
        self.motion_pub = self.create_publisher(Float64MultiArray, "/motion_matrix", 10)

        self.color_sub = self.create_subscription(String, "/colors", self.color_callback, 10)
        self.goal_done_sub = self.create_subscription(Bool, "/goal_done", self.goal_done_callback, 10)

        self.joint_names = [
            "base_link_to_link1",
            "link1_to_link2",
            "link2_to_link3",
            "link3_to_gripper_link",
        ]

        self.poses = {
            "home": [0.00, 0.00, 1.50, 0.00],
            "pre_pick": [0.00, -0.70, 2.00, 1.20],
            "pick": [0.00, -0.70, 2.00, 0.30],
            "lift": [0.00, 0.00, 1.50, 0.30],

            "pre_place_red": [1.20, 0.00, 1.50, 0.30],
            "place_red": [1.20, 0.00, 1.50, 1.20],
            "retreat_red": [1.20, 0.00, 1.50, 1.20],

            "pre_place_green": [-1.20, 0.00, 1.50, 0.30],
            "place_green": [-1.20, 0.00, 1.50, 1.20],
            "retreat_green": [-1.20, 0.00, 1.50, 1.20],
        }

        self.sequence = []
        self.current_index = 0

        self.state = "idle"
        self.busy = False

        self.saved_color = None
        self.sequence_part = 0
        self.waiting_for_goal_done = False
        self.goal_done_received = False

        self.current_position = self.poses["home"][:]
        self.start_position = self.poses["home"][:]
        self.target_position = self.poses["home"][:]

        self.motion_elapsed = 0.0
        self.hold_elapsed = 0.0
        self.motion_duration = 0.0
        self.hold_duration = 0.0
        self.current_action = "move"

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.update)

        self.get_logger().info("PickPlaceIsaacSim started.")

    def color_callback(self, msg):
        color = msg.data.strip().lower()

        if color == "yellow":
            color = "green"

        if color not in ["red", "green"]:
            return

        if self.busy or self.waiting_for_goal_done:
            return

        self.saved_color = color
        self.goal_done_received = False

        self.start_first_part(color)

    def goal_done_callback(self, msg):
        if not msg.data:
            return

        self.goal_done_received = True

        if self.busy:
            return

        if self.waiting_for_goal_done and self.saved_color is not None:
            self.start_second_part(self.saved_color)

    def start_first_part(self, color):
        self.sequence_part = 1
        self.waiting_for_goal_done = False

        if color == "red":
            steps = [
                ("home", 0.8, 0.2, 0),
                ("pre_pick", 1.0, 0.3, 0),
                ("pick", 0.8, 0.3, 0),
                ("pick", 0.1, 0.5, 1),
                ("lift", 0.8, 0.3, 0),
                ("pre_place_red", 1.0, 0.3, 0),
            ]
        else:
            steps = [
                ("home", 0.8, 0.2, 0),
                ("pre_pick", 1.0, 0.3, 0),
                ("pick", 0.8, 0.3, 0),
                ("pick", 0.1, 0.5, 1),
                ("lift", 0.8, 0.3, 0),
                ("pre_place_green", 1.0, 0.3, 0),
            ]

        self.load_steps(steps)
        self.publish_motion_matrix(steps)

    def start_second_part(self, color):
        self.sequence_part = 2
        self.waiting_for_goal_done = False

        if color == "red":
            steps = [
                ("place_red", 0.8, 0.3, 0),
                ("place_red", 0.1, 0.5, 2),
                ("retreat_red", 0.8, 0.3, 0),
                ("home", 1.0, 0.3, 0),
            ]
        else:
            steps = [
                ("place_green", 0.8, 0.3, 0),
                ("place_green", 0.1, 0.5, 2),
                ("retreat_green", 0.8, 0.3, 0),
                ("home", 1.0, 0.3, 0),
            ]

        self.load_steps(steps)
        self.publish_motion_matrix(steps)

    def publish_motion_matrix(self, steps):
        matrix = []

        for pose_name, move_time, hold_time, action_code in steps:
            row = self.poses[pose_name] + [
                move_time,
                hold_time,
                float(action_code),
            ]
            matrix.extend(row)

        msg = Float64MultiArray()
        msg.data = matrix
        self.motion_pub.publish(msg)

        self.get_logger().info("Published /motion_matrix once")

    def load_steps(self, steps):
        self.sequence = []

        for pose_name, move_time, hold_time, action_code in steps:
            target_pose = self.poses[pose_name][:]

            if action_code == 1:
                action = "grip_close"
            elif action_code == 2:
                action = "grip_open"
            else:
                action = "move"

            self.sequence.append((target_pose, move_time, hold_time, action))

        self.current_index = 0
        self.busy = True
        self.start_new_step()

    def start_new_step(self):
        if not self.sequence:
            self.finish_cycle()
            return

        target_pose, move_time, hold_time, action = self.sequence[self.current_index]

        self.start_position = self.current_position[:]
        self.target_position = target_pose[:]

        self.motion_duration = move_time
        self.hold_duration = hold_time
        self.motion_elapsed = 0.0
        self.hold_elapsed = 0.0
        self.current_action = action
        self.state = "moving"

    def lerp(self, a, b, t):
        return [ai + (bi - ai) * t for ai, bi in zip(a, b)]

    def publish_joint_state(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.joint_pub.publish(msg)

    def finish_cycle(self):
        if self.sequence_part == 1:
            self.busy = False
            self.sequence = []
            self.current_index = 0
            self.state = "waiting_goal_done"
            self.waiting_for_goal_done = True

            if self.goal_done_received and self.saved_color is not None:
                self.start_second_part(self.saved_color)

            return

        if self.sequence_part == 2:
            self.busy = False
            self.sequence = []
            self.current_index = 0
            self.state = "idle"

            self.sequence_part = 0
            self.saved_color = None
            self.waiting_for_goal_done = False
            self.goal_done_received = False
            self.current_position = self.poses["home"][:]

            return

    def update(self):
        if self.state in ["idle", "waiting_goal_done"]:
            self.publish_joint_state(self.current_position)
            return

        if self.state == "moving":
            if self.motion_duration <= 0.0:
                alpha = 1.0
            else:
                alpha = min(self.motion_elapsed / self.motion_duration, 1.0)

            self.current_position = self.lerp(
                self.start_position,
                self.target_position,
                alpha,
            )

            self.publish_joint_state(self.current_position)
            self.motion_elapsed += self.timer_period

            if alpha >= 1.0:
                self.state = "holding"
                self.hold_elapsed = 0.0

        elif self.state == "holding":
            self.publish_joint_state(self.current_position)
            self.hold_elapsed += self.timer_period

            if self.hold_elapsed >= self.hold_duration:
                self.current_index += 1

                if self.current_index >= len(self.sequence):
                    self.finish_cycle()
                else:
                    self.start_new_step()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceIsaacSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()