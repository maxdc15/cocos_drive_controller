#!/usr/bin/env python3
# line_error_node.py
import rclpy
from rclpy.node import Node

import cv2 as cv
import numpy as np
from std_msgs.msg import Float32


def gstreamer_pipeline(
    capture_width=400,
    capture_height=300,
    display_width=400,
    display_height=300,
    framerate=30,
    flip_method=2,
):
    return (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink"
    )


class LineErrorNode(Node):
    """Detecta la línea, calcula error horizontal y lo publica en /line_error"""

    def __init__(self):
        super().__init__('line_detection')

        # ── parámetros configurables ───────────────────────────
        self.declare_parameter('thresh',        60)   # Umbral binarización
        self.declare_parameter('roi_height',   0.25)  # Fracción inferior a analizar
        self.declare_parameter('rate',         30.0)  # [Hz]

        self.thresh      = self.get_parameter('thresh').value
        self.roi_height  = self.get_parameter('roi_height').value
        self.dt          = 1.0 / self.get_parameter('rate').value

        # Publisher
        self.pub_err = self.create_publisher(Float32, '/line_error', 10)

        # Cámara Jetson
        self.cap = cv.VideoCapture(0)
        #self.cap = cv.VideoCapture(gstreamer_pipeline(), cv.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara con GStreamer')
            rclpy.shutdown()
            raise RuntimeError('Camera open failed')

        self.timer = self.create_timer(self.dt, self.process_frame)
        self.get_logger().info('Nodo line_error_node iniciado')

    # ──────────────────────────────────────────────────────────
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Frame no válido')
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

        msg = Float32()
        msg.data = float(error_norm)
        self.pub_err.publish(msg)

        # # --- DEBUG opcional: comenta si no lo usas ---
        # cv.circle(roi, (cx, int(M['m01']/M['m00'])), 4, (0,255,0), -1)
        # cv.putText(roi, f"err: {error_norm:+.2f}", (10,20),
        #            cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
        # cv.imshow('ROI', roi)
        # if cv.waitKey(1) == ord('q'):
        #     rclpy.shutdown()

    # ──────────────────────────────────────────────────────────
    def destroy_node(self):
        self.cap.release()
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
