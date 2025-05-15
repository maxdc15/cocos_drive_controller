#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from rclpy.clock import Clock
from rclpy.duration import Duration
from enum import Enum, auto

class StateFSM(Enum):
    FOLLOW_LINE = auto()
    STOP        = auto()
    REDUCE_VEL  = auto()

class CmdRobotNode(Node):
    def __init__(self):
        super().__init__('cmd_robot_node')
        self.get_logger().info('Nodo cmd_robot_node iniciado')

        # ── Parámetros ───────────────────────────────────────────
        self.declare_parameter('v_max',     0.18)
        self.declare_parameter('v_min',     0.05)
        self.declare_parameter('kp_ang',    0.6)
        self.declare_parameter('threshold', 0.05)
        self.declare_parameter('timeout',   0.5)
        self.declare_parameter('rate',     50.0)

        self.v_max   = self.get_parameter('v_max').value
        self.v_min   = self.get_parameter('v_min').value
        self.kp_ang  = self.get_parameter('kp_ang').value
        self.thresh  = self.get_parameter('threshold').value
        self.timeout = self.get_parameter('timeout').value
        self.dt      = 1.0 / self.get_parameter('rate').value

        # ── Publisher de Twist ───────────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Subscriptions ────────────────────────────────────────
        self.create_subscription(Float32, '/line_error',      self.on_line_error,     10)
        self.create_subscription(Bool,    '/stop',            self.on_stop_signal,    10)
        self.create_subscription(Bool,    '/reduce_velocity', self.on_reduce_signal,  10)
        self.create_subscription(Bool,    '/follow_line',     self.on_follow_signal,  10)

        # ── Estado interno ───────────────────────────────────────
        self.error_line   = None
        self.state        = StateFSM.FOLLOW_LINE
        self.last_msg_time = Clock().now()

        # ── Timer de control ─────────────────────────────────────
        self.create_timer(self.dt, self.control_loop)

    # ── Callbacks de entrada ──────────────────────────────────
    def on_line_error(self, msg: Float32):
        self.error_line   = msg.data
        self.last_msg_time = Clock().now()

    def on_stop_signal(self, msg: Bool):
        if msg.data:
            self.state = StateFSM.STOP

    def on_reduce_signal(self, msg: Bool):
        if msg.data:
            self.state = StateFSM.REDUCE_VEL

    def on_follow_signal(self, msg: Bool):
        if msg.data:
            self.state = StateFSM.FOLLOW_LINE

    # ── Cálculo de Twist según parámetros ────────────────────
    def move_robot(self, vmax, vmin, kp_ang) -> Twist:
        twist = Twist()
        err = abs(self.error_line)
        if err <= self.thresh:
            twist.linear.x  = vmax
            twist.angular.z = 0.0
        else:
            twist.angular.z = -kp_ang * self.error_line
            scale = (err - self.thresh) / (1.0 - self.thresh)
            twist.linear.x = max(vmin, vmax * (1.0 - scale))
        
        self.pub_cmd.publish(twist)

    # ── Bucle principal de control ────────────────────────────
    def control_loop(self):
        # Watchdog: parar si no llegan datos de error
        if (Clock().now() - self.last_msg_time) > Duration(seconds=self.timeout):
            self.pub_cmd.publish(Twist())
            return

        # Si no tengo error o no hay evento, no hago nada
        if self.error_line is None:
            return

        # Ejecutar acción según estado
        if self.state == StateFSM.STOP:
            self.pub_cmd.publish(Twist())

        elif self.state == StateFSM.REDUCE_VEL:
            self.move_robot(self.v_max/2.0, self.v_min/2.0, self.kp_ang/2.0)
            

        elif self.state == StateFSM.FOLLOW_LINE:
            self.move_robot(self.v_max, self.v_min, self.kp_ang)
        

        # Resetear para esperar nueva señal
        self.state = None

def main(args=None):
    rclpy.init(args=args)
    node = CmdRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
