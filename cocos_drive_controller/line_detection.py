#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

import cv2 as cv
import numpy as np
from cv_bridge import CvBridge


class LineErrorNode(Node):
    """Detecta la línea desde una imagen de ROS y publica el error horizontal en /line_error"""

    def __init__(self):
        super().__init__('line_detection')

        # ── parámetros configurables ───────────────────────────
        self.declare_parameter('thresh',        60)   # Umbral binarización
        self.declare_parameter('roi_height',   0.25)  # Fracción inferior a analizar

        self.thresh      = self.get_parameter('thresh').value
        self.roi_height  = self.get_parameter('roi_height').value

        self.bridge = CvBridge()

        # Publisher
        self.pub_err = self.create_publisher(Float32, '/line_error', 10)

        # Subscriber a las imágenes del tópico /video_source/raw
        self.sub_img = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)

        self.get_logger().info('Nodo line_error_node iniciado con entrada desde /video_source/raw')

    # ──────────────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')
            return

        h, w = frame.shape[:2]
        roi = frame[int(h * (1 - self.roi_height)) : h, :]

        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        _, bin_img = cv.threshold(gray, self.thresh, 255, cv.THRESH_BINARY_INV)

        contours, _ = cv.findContours(bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        biggest = max(contours, key=cv.contourArea)
        M = cv.moments(biggest)
        if M['m00'] == 0:
            return

        cx = int(M['m10'] / M['m00'])           
        error_norm = (cx - w // 2) / (w // 2)  

        msg_err = Float32()
        msg_err.data = float(error_norm)
        self.pub_err.publish(msg_err)

        # # --- DEBUG opcional: comenta si no lo usas ---
        # cv.circle(roi, (cx, int(M['m01']/M['m00'])), 4, (0,255,0), -1)
        # cv.putText(roi, f"err: {error_norm:+.2f}", (10,20),
        #            cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
        # cv.imshow('ROI', roi)
        # if cv.waitKey(1) == ord('q'):
        #     rclpy.shutdown()

    # ──────────────────────────────────────────────────────────
    def destroy_node(self):
        cv.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = LineErrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()
