import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32

class RoadCamera(Node):
    def __init__(self, robot, timestep):
        super().__init__('road_camera_node')
        self.i = 0
        self.width = 512
        self.height = 16

        self.camera = robot.getDevice('road_camera')
        self.camera.enable(timestep)
        self.timestep = timestep

        self.lane_error_pub = self.create_publisher(Float32, '/lane_error', 10)

        self.get_logger().info("Road camera device initialized.")

        self.create_timer(0.5, self.timer_callback)

    def get_image(self):
        image = self.camera.getImage()
        return image  # raw bytes
    
    def get_center(self, image):
        img = np.frombuffer(image, np.uint8).reshape((self.height, self.width, 4))
        means = img.mean(axis=(0, 2))

        threshold = means.max() * 0.7

        if (np.sum(means > threshold) > 256):
            center = 256
        else:
            center = np.argmax(means)

        # self.get_logger().info(f'Columnas: {np.sum(means >= threshold)} Threshold: {threshold}')
        
        ideal_center = self.width / 2

        return (center - ideal_center) / ideal_center
    
    def timer_callback(self):
        self.i += 1
        image = self.get_image()

        center = self.get_center(image)

        msg = Float32()
        msg.data = float(center)
        self.lane_error_pub.publish(msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = RoadCamera()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
