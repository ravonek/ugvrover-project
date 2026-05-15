#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    t: float


class SimRealOdomFollower(Node):
    def __init__(self):
        super().__init__("sim_real_odom_follower")

        self.declare_parameter("cmd_in_topic", "/cmd_vel_sim")
        self.declare_parameter("cmd_real_topic", "/cmd_vel")
        self.declare_parameter("odom_sim_topic", "/odom_sim")
        self.declare_parameter("odom_real_topic", "/odom")

        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("cmd_timeout", 0.4)
        self.declare_parameter("odom_timeout", 1.0)

        self.declare_parameter("real_linear_gain", 0.50)
        self.declare_parameter("real_angular_gain", 0.66)

        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("kp_angular", 1.0)

        self.declare_parameter("linear_deadband", 0.03)
        self.declare_parameter("angular_deadband", 0.05)

        self.declare_parameter("max_linear", 0.22)
        self.declare_parameter("max_angular", 0.55)

        self.declare_parameter("max_linear_correction", 0.04)
        self.declare_parameter("max_angular_correction", 0.12)

        self.declare_parameter("sim_min_linear_speed", 0.005)
        self.declare_parameter("sim_min_angular_speed", 0.02)

        self.declare_parameter("linear_cmd_eps", 0.001)
        self.declare_parameter("angular_cmd_eps", 0.001)

        self.cmd_in_topic = self.get_parameter("cmd_in_topic").value
        self.cmd_real_topic = self.get_parameter("cmd_real_topic").value
        self.odom_sim_topic = self.get_parameter("odom_sim_topic").value
        self.odom_real_topic = self.get_parameter("odom_real_topic").value

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)

        self.real_linear_gain = float(self.get_parameter("real_linear_gain").value)
        self.real_angular_gain = float(self.get_parameter("real_angular_gain").value)

        self.kp_linear = float(self.get_parameter("kp_linear").value)
        self.kp_angular = float(self.get_parameter("kp_angular").value)

        self.linear_deadband = float(self.get_parameter("linear_deadband").value)
        self.angular_deadband = float(self.get_parameter("angular_deadband").value)

        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        self.max_linear_correction = float(
            self.get_parameter("max_linear_correction").value
        )
        self.max_angular_correction = float(
            self.get_parameter("max_angular_correction").value
        )

        self.sim_min_linear_speed = float(
            self.get_parameter("sim_min_linear_speed").value
        )
        self.sim_min_angular_speed = float(
            self.get_parameter("sim_min_angular_speed").value
        )

        self.linear_cmd_eps = float(self.get_parameter("linear_cmd_eps").value)
        self.angular_cmd_eps = float(self.get_parameter("angular_cmd_eps").value)

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0

        self.sim_pose: Optional[Pose2D] = None
        self.prev_sim_pose: Optional[Pose2D] = None
        self.real_pose: Optional[Pose2D] = None

        self.sim_linear_speed = 0.0
        self.sim_angular_speed = 0.0

        self.motion_active = False
        self.start_sim_pose: Optional[Pose2D] = None
        self.start_real_pose: Optional[Pose2D] = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_real_topic, 10)

        self.create_subscription(Twist, self.cmd_in_topic, self.cmd_callback, 10)
        self.create_subscription(Odometry, self.odom_sim_topic, self.odom_sim_callback, 20)
        self.create_subscription(Odometry, self.odom_real_topic, self.odom_real_callback, 20)

        self.create_timer(1.0 / self.control_rate, self.timer_callback)

        self.get_logger().info("sim_real_odom_follower started")
        self.get_logger().info(f"cmd_in_topic={self.cmd_in_topic}")
        self.get_logger().info(f"cmd_real_topic={self.cmd_real_topic}")
        self.get_logger().info(f"odom_sim_topic={self.odom_sim_topic}")
        self.get_logger().info(f"odom_real_topic={self.odom_real_topic}")
        self.get_logger().info(f"real_linear_gain={self.real_linear_gain}")
        self.get_logger().info(f"real_angular_gain={self.real_angular_gain}")
        self.get_logger().info(f"kp_linear={self.kp_linear}")
        self.get_logger().info(f"kp_angular={self.kp_angular}")
        self.get_logger().info(f"max_linear_correction={self.max_linear_correction}")
        self.get_logger().info(f"max_angular_correction={self.max_angular_correction}")

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def odom_sim_callback(self, msg: Odometry):
        new_pose = self.odom_to_pose(msg)

        if self.sim_pose is not None:
            dt = new_pose.t - self.sim_pose.t

            if dt > 0.001:
                dx = new_pose.x - self.sim_pose.x
                dy = new_pose.y - self.sim_pose.y
                dyaw = normalize_angle(new_pose.yaw - self.sim_pose.yaw)

                self.sim_linear_speed = math.sqrt(dx * dx + dy * dy) / dt
                self.sim_angular_speed = abs(dyaw) / dt

        self.prev_sim_pose = self.sim_pose
        self.sim_pose = new_pose

    def odom_real_callback(self, msg: Odometry):
        self.real_pose = self.odom_to_pose(msg)

    def odom_to_pose(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        return Pose2D(
            x=p.x,
            y=p.y,
            yaw=quat_to_yaw(q),
            t=time.time()
        )

    def cmd_is_active(self, cmd: Twist):
        return (
            abs(cmd.linear.x) > self.linear_cmd_eps or
            abs(cmd.angular.z) > self.angular_cmd_eps
        )

    def odom_ready(self):
        now = time.time()

        if self.sim_pose is None:
            return False

        if self.real_pose is None:
            return False

        if now - self.sim_pose.t > self.odom_timeout:
            return False

        if now - self.real_pose.t > self.odom_timeout:
            return False

        return True

    def start_motion_if_needed(self):
        if self.motion_active:
            return

        self.motion_active = True
        self.start_sim_pose = self.sim_pose
        self.start_real_pose = self.real_pose

        self.get_logger().info("New motion segment started")

    def stop_motion(self):
        self.cmd_pub.publish(Twist())
        self.motion_active = False
        self.start_sim_pose = None
        self.start_real_pose = None

    def signed_linear_delta(self, start: Pose2D, current: Pose2D, sign_reference: float):
        dx = current.x - start.x
        dy = current.y - start.y

        distance = math.sqrt(dx * dx + dy * dy)

        if sign_reference < 0.0:
            return -distance

        return distance

    def get_errors(self, cmd: Twist):
        if self.start_sim_pose is None or self.start_real_pose is None:
            return 0.0, 0.0, 0.0, 0.0

        linear_sign = cmd.linear.x

        if abs(linear_sign) < self.linear_cmd_eps:
            linear_sign = self.last_cmd.linear.x

        sim_linear = self.signed_linear_delta(
            self.start_sim_pose,
            self.sim_pose,
            linear_sign
        )

        real_linear = self.signed_linear_delta(
            self.start_real_pose,
            self.real_pose,
            linear_sign
        )

        sim_yaw = normalize_angle(self.sim_pose.yaw - self.start_sim_pose.yaw)
        real_yaw = normalize_angle(self.real_pose.yaw - self.start_real_pose.yaw)

        linear_error = sim_linear - real_linear
        yaw_error = normalize_angle(sim_yaw - real_yaw)

        return linear_error, yaw_error, sim_linear, real_linear

    def limit_same_direction(self, value, requested, maximum):
        if requested > 0.0:
            return clamp(value, 0.0, maximum)

        if requested < 0.0:
            return clamp(value, -maximum, 0.0)

        return clamp(value, -maximum, maximum)

    def publish_live_control(self, cmd: Twist):
        out = Twist()

        linear_requested = abs(cmd.linear.x) > self.linear_cmd_eps
        angular_requested = abs(cmd.angular.z) > self.angular_cmd_eps

        linear_error, yaw_error, sim_linear, real_linear = self.get_errors(cmd)

        sim_linear_confirmed = self.sim_linear_speed >= self.sim_min_linear_speed
        sim_angular_confirmed = self.sim_angular_speed >= self.sim_min_angular_speed

        if linear_requested and sim_linear_confirmed:
            base_linear = cmd.linear.x * self.real_linear_gain
            correction_linear = self.kp_linear * linear_error

            out.linear.x = self.limit_same_direction(
                base_linear + correction_linear,
                cmd.linear.x,
                self.max_linear
            )

        else:
            out.linear.x = 0.0

        if angular_requested and sim_angular_confirmed:
            base_angular = cmd.angular.z * self.real_angular_gain
            correction_angular = self.kp_angular * yaw_error

            out.angular.z = self.limit_same_direction(
                base_angular + correction_angular,
                cmd.angular.z,
                self.max_angular
            )

        else:
            out.angular.z = 0.0

        self.cmd_pub.publish(out)

        self.get_logger().info(
            f"LIVE | sim_lin={sim_linear:.3f}, real_lin={real_linear:.3f}, "
            f"lin_err={linear_error:.3f}, yaw_err={math.degrees(yaw_error):.1f} deg, "
            f"cmd_out=({out.linear.x:.3f}, {out.angular.z:.3f})"
        )

        if linear_requested and not sim_linear_confirmed:
            self.get_logger().warn(
                f"Linear blocked: cmd={cmd.linear.x:.3f}, "
                f"sim_linear_speed={self.sim_linear_speed:.3f}"
            )

        if angular_requested and not sim_angular_confirmed:
            self.get_logger().warn(
                f"Angular blocked: cmd={cmd.angular.z:.3f}, "
                f"sim_angular_speed={self.sim_angular_speed:.3f}"
            )

    def publish_after_stop_correction(self):
        fake_cmd = self.last_cmd

        linear_error, yaw_error, sim_linear, real_linear = self.get_errors(fake_cmd)

        out = Twist()

        if abs(linear_error) > self.linear_deadband:
            linear_correction = self.kp_linear * linear_error

            if self.last_cmd.linear.x > self.linear_cmd_eps:
                linear_correction = clamp(
                    linear_correction,
                    0.0,
                    self.max_linear_correction
                )

            elif self.last_cmd.linear.x < -self.linear_cmd_eps:
                linear_correction = clamp(
                    linear_correction,
                    -self.max_linear_correction,
                    0.0
                )

            else:
                linear_correction = 0.0

            out.linear.x = linear_correction

        if abs(yaw_error) > self.angular_deadband:
            out.angular.z = clamp(
                self.kp_angular * yaw_error,
                -self.max_angular_correction,
                self.max_angular_correction
            )

        self.cmd_pub.publish(out)

        self.get_logger().info(
            f"CORRECT | sim_lin={sim_linear:.3f}, real_lin={real_linear:.3f}, "
            f"lin_err={linear_error:.3f}, yaw_err={math.degrees(yaw_error):.1f} deg, "
            f"cmd_out=({out.linear.x:.3f}, {out.angular.z:.3f})"
        )

        if out.linear.x == 0.0 and out.angular.z == 0.0:
            self.get_logger().info("Correction finished")
            self.stop_motion()

    def timer_callback(self):
        now = time.time()
        cmd = self.last_cmd

        cmd_fresh = (now - self.last_cmd_time) <= self.cmd_timeout
        active = cmd_fresh and self.cmd_is_active(cmd)

        if not self.odom_ready():
            self.get_logger().warn("Waiting for /odom_sim and /odom. Real robot stopped.")
            self.cmd_pub.publish(Twist())
            return

        if active:
            self.start_motion_if_needed()
            self.publish_live_control(cmd)
            return

        if self.motion_active:
            self.publish_after_stop_correction()
            return

        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SimRealOdomFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()