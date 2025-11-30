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
from webots_ros2_driver.webots_controller import WebotsController

class LaneErrorListener(Node):
    def __init__(self):
        super().__init__('lane_error_listener')
        self.lane_error = 0.0
        self.create_subscription(Float32, '/car/lane', self.lane_error_callback, 10)

    def lane_error_callback(self, msg):
        self.lane_error = msg.data
        self.get_logger().info(f"Lane Error: {self.lane_error}")

class SignalListener(Node):
    def __init__(self):
        super().__init__('signal_callback')
        self.signal = 0.0
        self.create_subscription(String, '/car/detected_sign', self.signal_callback, 10)

    def signal_callback(self, msg):
        self.signal = msg.data
        # self.get_logger().info(f"Signal: {self.signal}")

class VehicleController(WebotsController):
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

        self.base_speed = 55
        self.kp = 0.8
        self.ki = 0.05
        self.kd = 0.9
        self.max_angle = 0.1
        self.prev_error = 0
        self.sum_error = 0
        
        self.stop_signal_seen = False
        self.stop_wait_counter = 0
        self.stop_wait_steps = int(1000 / self.__timestep)
        
        self.zero_error_counter = 0
        self.required_zero_count = 52
        self.last_direction = 0
        self.line_lost_timer = 0
        self.line_lost_active = False

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.road_camera)
        self.executor.add_node(self.car_camera)
        self.executor.add_node(self.lane_error_listener)
        self.executor.add_node(self.signal_listener)

        self.logger = rclpy.logging.get_logger('vehicle_controller')
        self.logger.info("Cámaras inicializadas correctamente")
        

    def step(self):
        self.executor.spin_once(timeout_sec=0.01)

        lane_error = self.lane_error_listener.lane_error
        signal = self.signal_listener.signal
        
        speed = self.base_speed

        if signal == "STOP" and not self.stop_signal_seen:
            self.stop_signal_seen = True
            self.stop_wait_counter = self.stop_wait_steps
            speed = 0
        
        elif self.stop_signal_seen:
            if self.stop_wait_counter > 0:
                speed = 0
                self.stop_wait_counter -= 1
            else:
                speed = self.base_speed

        if signal != "STOP" and self.stop_signal_seen and self.stop_wait_counter == 0:
            self.stop_signal_seen = False

        if signal == 'YIELD':
            speed = self.base_speed * 0.5
        elif signal == 'SPEED_LIMIT':
            speed = 30.0

        self.logger.info(f"Signal: {signal} Speed: {speed}")
        self.__robot.setCruisingSpeed(speed)

        current_direction = 0
        if self.prev_error > 0:
            current_direction = 1
        elif self.prev_error < 0:
            current_direction = -1

        if current_direction != 0:
            self.last_direction = current_direction
            
        if 1 <= self.zero_error_counter < self.required_zero_count:
            if not getattr(self, "line_lost_active", False):
                self.line_lost_active = True
                self.line_lost_timer = 0
            self.line_lost_timer += 1
            
            if self.line_lost_timer >= int(2000 / self.__timestep):
                opposite_angle = -self.last_direction * self.max_angle
                self.__robot.setSteeringAngle(opposite_angle)
                self.logger.info(f"[AUTO-RECOVERY] Girando hacia el lado opuesto: {opposite_angle}")
                return

        if lane_error == 0.0:
            self.zero_error_counter += 1
        else:
            self.zero_error_counter = 0
            self.line_lost_timer = 0
            self.line_lost_active = False
        if self.zero_error_counter >= self.required_zero_count:
            alpha = 0.0
            self.prev_error = 0.0
            self.sum_error = 0.0
            self.__robot.setSteeringAngle(0.0)
            self.logger.info(f"Alpha = 0 (lane error 0 durante {self.zero_error_counter} ciclos)")
            return

        self.sum_error += lane_error
        self.sum_error = max(min(self.sum_error, 10), -10)

        d_error = lane_error - self.prev_error
        alpha = self.kp * lane_error + self.ki * self.sum_error + self.kd * d_error
        self.prev_error = lane_error

        
        alpha = max(min(alpha, self.max_angle), -self.max_angle)
        
        self.logger.info(f"Alpha: {alpha}")
        try:
            steering_angle = math.atan(
                1 / (1 / math.tan(alpha) - self.trackFront / (2 * self.wheelbase))
            )
        except ZeroDivisionError:
            steering_angle = 0.0

        steering_angle = max(min(steering_angle, self.max_angle), -self.max_angle)

        self.__robot.setSteeringAngle(steering_angle)
