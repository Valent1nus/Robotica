#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class RoadFollower(Node):
    def __init__(self, robot, timestep):
        super().__init__('road_follower')
        self.i = 0
        self.width = 512
        self.height = 16

        self.camera = robot.getDevice('road_camera')
        self.camera.enable(timestep)
        self.timestep = timestep
        
        self.publisher_ = self.create_publisher(Float32, '/car/lane', 10)
        self.get_logger().info("Road camera device initialized.")
        self.image_callback
        
    def get_image(self):
        image = self.camera.getImage()
        return image  # raw bytes

    def image_callback(self):
        image = self.get_image()
        img = np.frombuffer(image, np.uint8).reshape((self.height, self.width, 4))
        mean_cols = img.mean(axis=(0, 2))

        threshold = mean_cols.max() * 0.7

        
        if (np.sum(mean_cols > threshold) > 256):
            center_detected = 256
        else:
            center_detected = int(np.argmax(mean_cols))
        center_ideal = self.width // 2

        error = (center_detected - center_ideal) / center_ideal

        msg = Float32()
        msg.data = float(error)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoadFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

