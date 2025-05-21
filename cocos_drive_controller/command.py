#!/usr/bin/env python3
# line_controller_node.py
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.clock import Clock


class LineController(Node):
    def __init__(self):
        super().__init__('line_controller')

        # ── parámetros ───────────────────────────────────────────────
        self.declare_parameter('v_max',     0.20)   # velocidad recto   [m/s]
        self.declare_parameter('v_min',     0.05)   # mínima en curva   [m/s]
        self.declare_parameter('kp_ang',    0.8)    # ganancia angular  [rad/s]
        self.declare_parameter('threshold', 0.05)   # error bajo el cual se va recto
        self.declare_parameter('timeout',   0.5)    # detiene si no hay datos [s]

        self.v_max    = self.get_parameter('v_max').value
        self.v_min    = self.get_parameter('v_min').value
        self.kp_ang   = self.get_parameter('kp_ang').value
        self.thresh   = self.get_parameter('threshold').value
        self.timeout  = self.get_parameter('timeout').value

        # publishers / subscribers
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_err = self.create_subscription(Float32,
                                                '/line_error',
                                                self.error_cb,
                                                10)

        self.last_msg_time = Clock().now()
        self.current_err   = 0.0

        # timer de seguridad para parar el robot si no llega info
        self.create_timer(0.05, self.watchdog)

        self.get_logger().info('Nodo line_controller iniciado')

    # ────────────────────────────────────────────────────────────────
    def error_cb(self, msg: Float32):
        self.current_err = float(msg.data)
        self.last_msg_time = Clock().now()

        twist = Twist()

        # distancia al centro normalizada |error|
        abs_err = abs(self.current_err)

        if abs_err <= self.thresh:
            # recto
            twist.linear.x  = self.v_max
            twist.angular.z = 0.0
        else:
            # curva: giro ∝ error ; velocidad disminuye gradualmente
            twist.angular.z = - self.kp_ang * self.current_err   # signo: izquierda/-derecha
            # velocidad lineal mapea [threshold … 1]  →  [v_max … v_min]
            scale = (abs_err - self.thresh) / (1.0 - self.thresh)
            twist.linear.x = max(self.v_min,
                                 self.v_max * (1.0 - scale))

        self.pub_cmd.publish(twist)

    # ────────────────────────────────────────────────────────────────
    def watchdog(self):
        """Detiene el robot si no recibe error en <timeout> segundos."""
        if (Clock().now() - self.last_msg_time) > Duration(seconds=self.timeout):
            stop = Twist()
            self.pub_cmd.publish(stop)


def main():
    rclpy.init()
    node = LineController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

