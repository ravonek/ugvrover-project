#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


class CubeJointController(Node):
    def __init__(self):
        super().__init__("cube_joint_controller")

        # Isaac Sim gripper/grobik joint
        self.cube_pub = self.create_publisher(
            JointState,
            "/cube_joint_command",
            10
        )

        # Real UGV pan-tilt joint topic
        self.ugv_joint_pub = self.create_publisher(
            JointState,
            "/ugv/joint_states",
            10
        )

        self.sub = self.create_subscription(
            Bool,
            "/cube_sent_done",
            self.cube_sent_done_callback,
            10
        )

        self.cube_joint_names = [
            "Revolute_30_sim"
        ]

        self.ugv_joint_names = [
            "pt_base_link_to_pt_link1",
            "pt_link1_to_pt_link2"
        ]

        self.home_angle = 0.0

        # Isaac керек: -0.8
        self.cube_target_angle = -0.8

        # Real UGV керек: +0.8
        self.ugv_target_angle = 0.8

        self.current_cube_angle = self.home_angle
        self.current_ugv_angle = self.home_angle

        self.start_cube_angle = self.home_angle
        self.start_ugv_angle = self.home_angle

        self.goal_cube_angle = self.home_angle
        self.goal_ugv_angle = self.home_angle

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.update)

        self.state = "idle"
        self.elapsed = 0.0
        self.move_duration = 2.5
        self.hold_duration = 3.0
        self.hold_elapsed = 0.0

        self.busy = False
        self.last_cube_sent_done = False

        self.get_logger().info("CubeJointController started.")
        self.get_logger().info("Isaac: /cube_joint_command -> Revolute_30_sim = -0.8")
        self.get_logger().info("Real UGV: /ugv/joint_states -> pt_link1_to_pt_link2 = +0.8")

    def cube_sent_done_callback(self, msg):
        value = msg.data

        if value and not self.last_cube_sent_done:
            if self.busy:
                self.get_logger().info("Cube joint busy, ignoring trigger.")
            else:
                self.get_logger().info("/cube_sent_done=True received. Starting motion.")
                self.start_motion_to_target()

        self.last_cube_sent_done = value

    def start_motion_to_target(self):
        self.busy = True
        self.state = "moving_to_target"

        self.start_cube_angle = self.current_cube_angle
        self.start_ugv_angle = self.current_ugv_angle

        self.goal_cube_angle = self.cube_target_angle
        self.goal_ugv_angle = self.ugv_target_angle

        self.elapsed = 0.0
        self.hold_elapsed = 0.0

    def start_motion_home(self):
        self.state = "moving_home"

        self.start_cube_angle = self.current_cube_angle
        self.start_ugv_angle = self.current_ugv_angle

        self.goal_cube_angle = self.home_angle
        self.goal_ugv_angle = self.home_angle

        self.elapsed = 0.0

    def lerp(self, a, b, t):
        return a + (b - a) * t

    def publish_cube_joint(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.cube_joint_names
        msg.position = [self.current_cube_angle]
        self.cube_pub.publish(msg)

    def publish_ugv_joint(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.ugv_joint_names

        # base joint = 0.0, tilt joint = current_ugv_angle
        msg.position = [0.0, self.current_ugv_angle]

        self.ugv_joint_pub.publish(msg)

    def publish_all(self):
        self.publish_cube_joint()
        self.publish_ugv_joint()

    def update(self):
        if self.state == "idle":
            self.current_cube_angle = self.home_angle
            self.current_ugv_angle = self.home_angle
            self.publish_all()
            return

        if self.state == "moving_to_target":
            alpha = min(self.elapsed / self.move_duration, 1.0)

            self.current_cube_angle = self.lerp(
                self.start_cube_angle,
                self.goal_cube_angle,
                alpha
            )

            self.current_ugv_angle = self.lerp(
                self.start_ugv_angle,
                self.goal_ugv_angle,
                alpha
            )

            self.publish_all()
            self.elapsed += self.timer_period

            if alpha >= 1.0:
                self.get_logger().info("Reached target: Isaac=-0.8, Real UGV=+0.8")
                self.state = "holding"
                self.hold_elapsed = 0.0

            return

        if self.state == "holding":
            self.publish_all()
            self.hold_elapsed += self.timer_period

            if self.hold_elapsed >= self.hold_duration:
                self.get_logger().info("Returning both joints back to 0.0")
                self.start_motion_home()

            return

        if self.state == "moving_home":
            alpha = min(self.elapsed / self.move_duration, 1.0)

            self.current_cube_angle = self.lerp(
                self.start_cube_angle,
                self.goal_cube_angle,
                alpha
            )

            self.current_ugv_angle = self.lerp(
                self.start_ugv_angle,
                self.goal_ugv_angle,
                alpha
            )

            self.publish_all()
            self.elapsed += self.timer_period

            if alpha >= 1.0:
                self.current_cube_angle = self.home_angle
                self.current_ugv_angle = self.home_angle
                self.publish_all()

                self.state = "idle"
                self.busy = False
                self.get_logger().info("Both joints returned home.")

            return


def main(args=None):
    rclpy.init(args=args)

    node = CubeJointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()