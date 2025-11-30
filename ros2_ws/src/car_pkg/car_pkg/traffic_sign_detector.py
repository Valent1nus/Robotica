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
    # Conservamos la firma con robot y timestep para compatibilidad con tu controller,
    # pero NO utilizamos robot/enable aquí para evitar conflictos con Webots.
    def __init__(self, robot=None, timestep=None):
        super().__init__('traffic_sign_detector')
        self.bridge = CvBridge()

        # Nos suscribimos a la imagen publicada por webots_ros2_driver
        self.subscription = self.create_subscription(
            Image, '/car/car_camera/image_color', self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, '/car/detected_sign', 10)

        # Cargar plantillas correctamente
        pkg_route = os.path.dirname(__file__)
        base_path = os.path.join(os.path.dirname(pkg_route), 'signs')
        self.templates = {}
        expected = ['stop', 'yield', 'speed_limit']
        for name in expected:
            path = os.path.join(base_path, f"{name}.png")
            if os.path.exists(path):
                img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
                if img is None:
                    self.get_logger().warn(f"Plantilla {name} existe pero cv2.imread devolvió None: {path}")
                    continue
                # Convertimos a gris y, si tiene alpha, removemos fondo
                if img.shape[-1] == 4:
                    # si tiene canal alpha, combinar con fondo blanco antes de gris
                    alpha = img[..., 3] / 255.0
                    rgb = img[..., :3].astype(np.float32)
                    bg = 255.0 * (1.0 - alpha[..., None])
                    composited = rgb * alpha[..., None] + bg
                    gray = cv2.cvtColor(composited.astype(np.uint8), cv2.COLOR_BGR2GRAY)
                else:
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                self.templates[name.upper()] = gray
                self.get_logger().info(f"Plantilla cargada: {name} -> {path} size={gray.shape}")
            else:
                self.get_logger().warn(f"No existe plantilla: {path}")

        if len(self.templates) == 0:
            self.get_logger().error("No se cargó NINGUNA plantilla. Revisa rutas y archivos en /signs.")

        # parámetros
        self.threshold = 0.75
        self.scales = np.linspace(0.4, 1.2, 9)  # multi-escala para robustez

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error convertendo imgmsg a cv2: {e}")
            return

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detected = "NONE"
        best_overall = ('', 0.0)  # (name, max_val)

        for name, template in self.templates.items():
            h0, w0 = template.shape[:2]
            best_val_for_template = 0.0
            # iterar escalas (redimensionar plantilla)
            for s in self.scales:
                tw = max(1, int(w0 * s))
                th = max(1, int(h0 * s))
                if tw >= frame_gray.shape[1] or th >= frame_gray.shape[0]:
                    continue
                resized = cv2.resize(template, (tw, th), interpolation=cv2.INTER_AREA)
                res = cv2.matchTemplate(frame_gray, resized, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                if max_val > best_val_for_template:
                    best_val_for_template = max_val
                    best_loc = max_loc
                    best_size = (tw, th)
            self.get_logger().debug(f"Template {name}: best_val={best_val_for_template:.3f}")
            if best_val_for_template > best_overall[1]:
                best_overall = (name, best_val_for_template)

        if best_overall[1] >= self.threshold:
            detected = best_overall[0]
        else:
            detected = "NONE"

        # Publicar y loggear (info para debug)
        out = String()
        out.data = detected
        self.publisher_.publish(out)
        self.get_logger().info(f"Señal detectada: {detected}  (best={best_overall[0]} val={best_overall[1]:.3f})")

