# Nodo ROS 2 que recibe imágenes desde /video_source/raw, detecta la línea
# dentro de una región de interés (ROI) y publica el error lateral
# normalizado en /line_error (Float32).  Un error de +1 significa que la
# línea está en el extremo derecho del ROI; –1, en el extremo izquierdo.

import rclpy
from rclpy.node import Node

import cv2 as cv
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineErrorNode(Node):
    """Calcula el error horizontal de la línea y lo publica."""

    def __init__(self) -> None:
        super().__init__('line_error_node')
        self.get_logger().info('Line error node initialized')

        # ── Parámetros configurables (ajustables con ros2 param) ─────────
        self.declare_parameter('thresh',      60)   # (no se usa: Otsu)
        self.declare_parameter('roi_height', 0.20)  # % inferior de la imagen
        self.declare_parameter('roi_width',  0.50)  # % ancho centrado a usar
        self.declare_parameter('rate',       30.0)  # (no se usa aquí)

        self.thresh      = self.get_parameter('thresh').value
        self.roi_height  = self.get_parameter('roi_height').value
        self.roi_width   = self.get_parameter('roi_width').value

        self.pub_err = self.create_publisher(Float32, '/line_error', 10)
        self.bridge  = CvBridge()
        self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)

    def image_callback(self, msg: Image) -> None:
        """Convierte la imagen, extrae la línea y publica el error."""
        # 1) Convertir imagen ROS ⇢ OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")
            return

        # 2) Definir región de interés (ROI) -----------------------------
        h, w = frame.shape[:2]

        # ROI vertical: franja inferior (p. ej. 20 % de la altura)
        roi_h = frame[int(h * (1 - self.roi_height)) : h, :]

        # ROI horizontal: ventana centrada (p. ej. 50 % del ancho)
        roi_w      = int(w * self.roi_width)
        start_x    = (w - roi_w) // 2
        end_x      = start_x + roi_w
        roi        = roi_h[:, start_x:end_x]        # recorte final

        # 3) Pre-proceso: gris + binarización (Otsu) ----------------------
        gray    = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        _, bin_img = cv.threshold(
            gray, 0, 255,
            cv.THRESH_BINARY_INV + cv.THRESH_OTSU)  # línea = blanco (255)

        # 4) Contorno más grande = línea ---------------------------------
        contours, _ = cv.findContours(
            bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        biggest = max(contours, key=cv.contourArea)
        M = cv.moments(biggest)
        if M['m00'] == 0:                          # evita división por cero
            return

        # 5) Cálculo del error normalizado [-1, 1] -----------------------
        cx      = int(M['m10'] / M['m00'])        # coordenada X del centroide
        w_roi   = roi.shape[1]
        error   = (cx - w_roi // 2) / (w_roi // 2)

        # 6) Publicar -----------------------------------------------------
        self.pub_err.publish(Float32(data=float(error)))

        # DEBUG opcional: des-comentar para visualizar la ROI ------------
        # cv.circle(roi, (cx, int(M['m01']/M['m00'])), 4, (0,255,0), -1)
        # cv.putText(roi, f"err: {error:+.2f}", (10,20),
        #            cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
        # cv.imshow("ROI", roi)
        # cv.waitKey(1)

    def destroy_node(self) -> None:
        """Cierra cualquier ventana OpenCV (si se usó) y destruye el nodo."""
        cv.destroyAllWindows()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LineErrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nNode terminated by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
