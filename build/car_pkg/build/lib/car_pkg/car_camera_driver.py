import os

import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from std_msgs.msg import String

class CarCamera(Node):
    def __init__(self, robot, timestep):
        super().__init__('car_camera_node')
        self.i = 0

        self.camera = robot.getDevice('car_camera')
        self.camera.enable(timestep)
        self.timestep = timestep

        # Publicador de la señal detectada
        self.sign_pub = self.create_publisher(String, '/traffic_sign', 10)

        # Cargar plantillas de señales -------------------------
        base_path = os.path.dirname("/home/kinz/Desktop/Robotica/CarProject/car_pkg/")
        signs_path = os.path.join(base_path, 'signs')

        self.templates = {
            'yield': cv2.imread(os.path.join(signs_path, 'yield.png'),
                                cv2.IMREAD_COLOR),
            'speed_65': cv2.imread(os.path.join(signs_path, 'speed.png'),
                                   cv2.IMREAD_COLOR),
            'stop': cv2.imread(os.path.join(signs_path, 'stop.png'),
                               cv2.IMREAD_COLOR),
        }

        # El valor mínimo de confianza para aceptar una señal
        self.threshold = 0.6

        self.get_logger().info("Car camera device initialized.")

        self.create_timer(0.5, self.timer_callback)

    def get_image(self):
        width = self.camera.getWidth()
        height = self.camera.getHeight()

        # Webots devuelve BGRA
        img = np.frombuffer(self.camera.getImage(), np.uint8).reshape(
            (height, width, 4)
        )
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img
    
    # ---------------------------------------------------------
    # Template matching para todas las señales
    # ---------------------------------------------------------
    def detect_sign(self, frame):
        best_label = None
        best_score = 0.0

        for label, tmpl in self.templates.items():
            if tmpl is None:
                continue  # por si alguna plantilla no se ha cargado

            # Ajuste opcional de tamaño si quieres escalar la plantilla
            # tmpl_resized = cv2.resize(tmpl, (0, 0), fx=0.5, fy=0.5)
            tmpl_resized = tmpl

            res = cv2.matchTemplate(frame, tmpl_resized,
                                    cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(res)

            if max_val > best_score:
                best_score = max_val
                best_label = label

        if best_score >= self.threshold:
            return best_label, best_score
        else:
            return None, best_score
    
    def timer_callback(self):
        self.i += 1
        frame = self.get_image()
        label, score = self.detect_sign(frame)

        if label is not None:
            msg = String()
            msg.data = label
            self.sign_pub.publish(msg)
            # self.get_logger().info(
            #     f"Señal detectada: {label} (confianza={score:.2f})"
            # )
        else:
            msg = String()
            msg.data = "none"
            self.sign_pub.publish(msg)
            # self.get_logger().info(
            #     f"Señal detectada: {label} (confianza={score:.2f})"
            # )

def main(args=None):
    try:
        rclpy.init(args=args)
        node = CarCamera()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()