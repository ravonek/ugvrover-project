#!/usr/bin/env python3

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node

import serial
from serial import SerialException

from std_msgs.msg import Float64MultiArray


serial_port = "/dev/ttyUSB0"


class RoarmMotionMatrixSerial(Node):
    def __init__(self):
        super().__init__("roarm_motion_matrix_serial")

        self.declare_parameter("serial_port", serial_port)
        self.declare_parameter("baud_rate", 115200)

        serial_port_name = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value

        try:
            self.serial_port = serial.Serial(serial_port_name, baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {serial_port_name}, baud={baud_rate}")

            start_data = json.dumps({"T": 605, "cmd": 0}) + "\n"
            self.serial_port.write(start_data.encode())
            time.sleep(0.1)

        except SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.serial_port = None
            return

        self.motion_sub = self.create_subscription(
            Float64MultiArray,
            "/motion_matrix",
            self.motion_callback,
            10
        )

        self.busy = False
        self.sequence = []
        self.current_index = 0

        self.current_joints = np.array([0.0, 0.0, 1.5, 0.0], dtype=np.float64)
        self.start_joints = self.current_joints.copy()
        self.target_joints = self.current_joints.copy()

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.update)

        self.state = "idle"
        self.motion_elapsed = 0.0
        self.hold_elapsed = 0.0
        self.motion_duration = 0.0
        self.hold_duration = 0.0
        self.current_action = "move"

        self.get_logger().info("Real RoArm waiting for /motion_matrix ...")

    def motion_callback(self, msg):
        if self.busy:
            self.get_logger().info("Ignoring /motion_matrix because real RoArm is busy.")
            return

        data = list(msg.data)

        if len(data) % 7 != 0:
            self.get_logger().error("Wrong matrix size. Each step must have 7 values.")
            return

        self.sequence = []

        for i in range(0, len(data), 7):
            j1 = data[i]
            j2 = data[i + 1]
            j3 = data[i + 2]
            j4 = data[i + 3]
            move_time = data[i + 4]
            hold_time = data[i + 5]
            action_code = int(data[i + 6])

            real_angles = self.convert_matrix_angles_to_roarm(j1, j2, j3, j4)

            if action_code == 1:
                action = "grip_close"
            elif action_code == 2:
                action = "grip_open"
            else:
                action = "move"

            self.sequence.append(
                (real_angles, move_time, hold_time, action)
            )

        self.current_index = 0
        self.busy = True

        self.get_logger().info(f"Received /motion_matrix: {len(self.sequence)} steps")
        self.start_new_step()

    def convert_matrix_angles_to_roarm(self, j1, j2, j3, j4):
        base = -j1
        shoulder = -j2
        elbow = j3
        hand = 3.1415926 - j4

        return np.array([base, shoulder, elbow, hand], dtype=np.float64)

    def start_new_step(self):
        if self.current_index >= len(self.sequence):
            self.finish_cycle()
            return

        target_angles, move_time, hold_time, action = self.sequence[self.current_index]

        self.start_joints = self.current_joints.copy()
        self.target_joints = target_angles.copy()

        self.motion_duration = move_time
        self.hold_duration = hold_time
        self.motion_elapsed = 0.0
        self.hold_elapsed = 0.0
        self.current_action = action

        self.state = "moving"

        self.get_logger().info(
            f"Step {self.current_index + 1}/{len(self.sequence)} | "
            f"action={action} | target={self.target_joints.tolist()}"
        )

    def update(self):
        if self.state == "idle":
            return

        if self.state == "moving":
            if self.motion_duration <= 0.0:
                alpha = 1.0
            else:
                alpha = min(self.motion_elapsed / self.motion_duration, 1.0)

            self.current_joints = self.start_joints + (
                self.target_joints - self.start_joints
            ) * alpha

            self.send_roarm_json(self.current_joints)

            self.motion_elapsed += self.timer_period

            if alpha >= 1.0:
                self.state = "holding"
                self.hold_elapsed = 0.0

                if self.current_action == "grip_close":
                    self.get_logger().info("GRIP CLOSE")
                elif self.current_action == "grip_open":
                    self.get_logger().info("GRIP OPEN")

        elif self.state == "holding":
            self.send_roarm_json(self.current_joints)

            self.hold_elapsed += self.timer_period

            if self.hold_elapsed >= self.hold_duration:
                self.current_index += 1
                self.start_new_step()

    def send_roarm_json(self, joints):
        base = float(joints[0])
        shoulder = float(joints[1])
        elbow = float(joints[2])
        hand = float(joints[3])

        data = json.dumps({
            "T": 102,
            "base": base,
            "shoulder": shoulder,
            "elbow": elbow,
            "hand": hand,
            "spd": 100,
            "acc": 10
        }) + "\n"

        try:
            self.serial_port.write(data.encode())
        except SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

    def finish_cycle(self):
        self.get_logger().info("Finished /motion_matrix cycle")

        self.sequence = []
        self.current_index = 0
        self.busy = False
        self.state = "idle"

    def destroy_node(self):
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RoarmMotionMatrixSerial()

    if node.serial_port is not None and node.serial_port.is_open:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

        node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()