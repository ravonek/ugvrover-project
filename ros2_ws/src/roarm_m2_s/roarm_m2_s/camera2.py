#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge

import cv2
import numpy as np


class ColorCubeDetector(Node):
    def __init__(self):
        super().__init__("color_cube_detector")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/rgb",
            self.image_callback,
            10
        )

        self.color_pub = self.create_publisher(String, "/colors", 10)

        self.min_area = 800
        self.center_box_size = 120

        self.last_published_color = ""

        self.get_logger().info("Color cube detector started.")
        self.get_logger().info("Publishing only to /colors")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        display = frame.copy()

        h, w = frame.shape[:2]
        cx_img = w // 2
        cy_img = h // 2

        half_box = self.center_box_size // 2
        x1 = cx_img - half_box
        y1 = cy_img - half_box
        x2 = cx_img + half_box
        y2 = cy_img + half_box

        cv2.rectangle(display, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(
            display,
            "CENTER",
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_lower1 = np.array([0, 120, 70])
        red_upper1 = np.array([10, 255, 255])

        red_lower2 = np.array([170, 120, 70])
        red_upper2 = np.array([180, 255, 255])

        green_lower = np.array([40, 70, 70])
        green_upper = np.array([85, 255, 255])

        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        red_mask = self.clean_mask(red_mask)
        green_mask = self.clean_mask(green_mask)

        detected_color = None

        if self.detect_and_check_center(
            display,
            red_mask,
            "red",
            (0, 0, 255),
            x1,
            y1,
            x2,
            y2
        ):
            detected_color = "red"

        elif self.detect_and_check_center(
            display,
            green_mask,
            "green",
            (0, 255, 0),
            x1,
            y1,
            x2,
            y2
        ):
            detected_color = "green"

        if detected_color in ["red", "green"]:
            if detected_color != self.last_published_color:
                msg_out = String()
                msg_out.data = detected_color
                self.color_pub.publish(msg_out)

                self.last_published_color = detected_color

                self.get_logger().info(f"Published to /colors: {detected_color}")

        else:
            self.last_published_color = ""

        cv2.imshow("Color Cube Detector", display)

        key = cv2.waitKey(1)
        if key == ord("q"):
            self.get_logger().info("Shutting down...")
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def clean_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def detect_and_check_center(self, image, mask, label, color_bgr, x1, y1, x2, y2):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        found_in_center = False

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h if h != 0 else 0

            if 0.7 <= aspect_ratio <= 1.3:
                cx = x + w // 2
                cy = y + h // 2

                cv2.rectangle(image, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.circle(image, (cx, cy), 4, color_bgr, -1)

                cv2.putText(
                    image,
                    label.upper(),
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color_bgr,
                    2
                )

                if x1 <= cx <= x2 and y1 <= cy <= y2:
                    cv2.putText(
                        image,
                        f"{label.upper()} IN CENTER",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        color_bgr,
                        3
                    )

                    found_in_center = True

        return found_in_center


def main(args=None):
    rclpy.init(args=args)

    node = ColorCubeDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()