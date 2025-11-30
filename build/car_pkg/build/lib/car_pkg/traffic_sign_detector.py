#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
import os

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, '/car/car_camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, '/car/detected_sign', 10)

        # Cargar dataset de plantillas
        self.templates = {}
        base_path = os.path.join(os.path.dirname(__file__), 'signs')
        for name in ['stop', 'yield', 'speed_limit']:
            path = os.path.join(base_path, f'{name}.png')
            if os.path.exists(path):
                self.templates[name] = cv2.imread(path, cv2.IMREAD_COLOR)

        self.threshold = 0.75  # valor empírico

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detected = None

        for name, template in self.templates.items():
            res = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(res)
            if max_val > self.threshold:
                detected = name.upper()
                break

        msg_out = String()
        msg_out.data = detected if detected else "NONE"
        self.publisher_.publish(msg_out)

        self.get_logger().info(f'Señal detectada: {msg_out.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

