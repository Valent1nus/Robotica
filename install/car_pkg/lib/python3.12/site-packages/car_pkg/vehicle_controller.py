#!/usr/bin/env python3
import math
import rclpy
import rclpy.logging
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
from std_msgs.msg import String
from car_pkg.traffic_sign_detector import TrafficSignDetector
from car_pkg.road_follower import RoadFollower
from rclpy.node import Node

class LaneErrorListener(Node):
    def __init__(self):
        super().__init__('lane_error_listener')
        self.lane_error = 0.0
        self.create_subscription(Float32, '/car/lane', self.lane_error_callback, 10)

    def lane_error_callback(self, msg):
        self.lane_error = msg.data
        # self.get_logger().info(f"Lane Error: {self.lane_error}")

class SignalListener(Node):
    def __init__(self):
        super().__init__('signal_callback')
        self.signal = 0.0
        self.create_subscription(String, '/car/detected_sign', self.signal_callback, 10)

    def signal_callback(self, msg):
        self.signal = msg.data
        # self.get_logger().info(f"Signal: {self.signal}")

class VehicleController(Node):
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        rclpy.init(args=None)

        self.road_camera = RoadFollower(self.__robot, self.__timestep)
        self.car_camera = TrafficSignDetector(self.__robot, self.__timestep)
        self.lane_error_listener = LaneErrorListener()
        self.signal_listener = SignalListener()

        self.wheelbase = 0.30
        self.trackFront = 0.20

        self.base_speed = 25.0
        self.kp = 0.2
        self.max_angle = 0.5

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.road_camera)
        self.executor.add_node(self.car_camera)
        self.executor.add_node(self.lane_error_listener)
        self.executor.add_node(self.signal_listener)

        self.logger = rclpy.logging.get_logger('car_driver')
        self.logger.info("Cámaras inicializadas correctamente")

    def step(self):
        self.executor.spin_once(timeout_sec=0.01)

        lane_error = self.lane_error_listener.lane_error

        signal = self.signal_listener.signal

        speed = self.base_speed

        if signal == 'stop':
            speed = 0
        elif signal == 'yield':
            speed = self.base_speed * 0.5
        elif signal == 'speed_limit':
            speed = 170
        elif signal == 'none':
            speed = self.base_speed

        self.logger.info(f"Signal: {signal} Speed: {speed}")
        
        self.__robot.setCruisingSpeed(speed)

        alpha = self.kp * lane_error

        alpha = max(min(alpha, self.max_angle), -self.max_angle)

        try:
            steering_angle = math.atan(
                1 / (1 / math.tan(alpha) - self.trackFront / (2 * self.wheelbase))
            )
        except ZeroDivisionError:
            steering_angle = 0.0

        steering_angle = max(min(steering_angle, self.max_angle), -self.max_angle)

        self.__robot.setSteeringAngle(steering_angle)

