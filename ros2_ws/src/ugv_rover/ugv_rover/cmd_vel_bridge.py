#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")

        self.declare_parameter("linear_gain", 1.0)
        self.declare_parameter("angular_gain", 1.0)

        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel_sim",
            self.cmd_callback,
            10
        )

        self.get_logger().info("cmd_vel_bridge started: /cmd_vel_sim -> /cmd_vel")

    def cmd_callback(self, msg):
        out = Twist()

        out.linear.x = msg.linear.x * self.linear_gain
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z

        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z * self.angular_gain

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()