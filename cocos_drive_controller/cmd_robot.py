# Nodo ROS 2 que genera comandos /cmd_vel a partir del error de línea y de
# tres flags booleanos publicados por el *MainController*:
#   • /follow_line      → velocidad normal
#   • /reduce_velocity  → velocidad reducida
#   • /stop             → parar
#
# La lógica es:
#   1. Si /stop = True           →  robot detenido
#   2. Si /reduce_velocity = True→  misma ley de control pero con v/2
#   3. Si /follow_line = True    →  ley de control normal
#
# Un watchdog detiene el robot si deja de recibir /line_error.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from rclpy.clock import Clock
from rclpy.duration import Duration
from enum import Enum, auto


# ---------- Estados internos de la FSM ----------
class StateFSM(Enum):
    FOLLOW_LINE = auto()
    STOP        = auto()
    REDUCE_VEL  = auto()


class CmdRobotNode(Node):
    """Convierte error de línea + flags en un Twist /cmd_vel."""

    def __init__(self):
        super().__init__('cmd_robot_node')
        self.get_logger().info('Nodo cmd_robot_node iniciado')

        # ── Parámetros configurables ------------------------------------
        self.declare_parameter('v_max',     0.20)   # m/s   velocidad máx
        self.declare_parameter('v_min',     0.05)   # m/s   velocidad mín
        self.declare_parameter('kp_ang',    0.8)    # ganancia angular
        self.declare_parameter('threshold', 0.05)   # error sin giro
        self.declare_parameter('timeout',   0.5)    # s     watchdog
        self.declare_parameter('rate',     50.0)    # Hz    lazo control

        self.v_max   = self.get_parameter('v_max').value
        self.v_min   = self.get_parameter('v_min').value
        self.kp_ang  = self.get_parameter('kp_ang').value
        self.thresh  = self.get_parameter('threshold').value
        self.timeout = self.get_parameter('timeout').value
        self.dt      = 1.0 / self.get_parameter('rate').value

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Float32, '/line_error',      self.on_line_error,    10)
        self.create_subscription(Bool,    '/stop',            self.on_stop_signal,   10)
        self.create_subscription(Bool,    '/reduce_velocity', self.on_reduce_signal, 10)
        self.create_subscription(Bool,    '/follow_line',     self.on_follow_signal, 10)

        # ── Estado interno ---------------------------------------------
        self.error_line    = None          # último error recibido
        self.state         = StateFSM.FOLLOW_LINE
        self.last_msg_time = Clock().now() # para watchdog

        # ── Timer de control -------------------------------------------
        self.create_timer(self.dt, self.control_loop)

    # Callbacks de suscripciones
    def on_line_error(self, msg: Float32):
        self.error_line    = msg.data
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

    # Genera y publica un Twist según parámetros pedidos
    def move_robot(self, vmax: float, vmin: float, kp_ang: float) -> None:
        twist = Twist()
        err = abs(self.error_line)

        # Zona muerta angular: avanza recto dentro del umbral
        if err <= self.thresh:
            twist.linear.x  = vmax
            twist.angular.z = 0.0
        else:
            # Giro proporcional al error
            twist.angular.z = -kp_ang * self.error_line
            # Reduce velocidad lineal a medida que aumenta el error
            scale = (err - self.thresh) / (1.0 - self.thresh)
            twist.linear.x = max(vmin, vmax * (1.0 - scale))

        self.pub_cmd.publish(twist)

    # Bucle de control (ejecutado cada dt)
    def control_loop(self) -> None:
        # Watchdog: detiene el robot si el error no llega a tiempo
        if (Clock().now() - self.last_msg_time) > Duration(seconds=self.timeout):
            self.pub_cmd.publish(Twist())  # Twist vacío = paro
            return

        # Si aún no se ha recibido ningún error, no se actúa
        if self.error_line is None:
            return

        # Selección de acción según estado actual
        if self.state == StateFSM.STOP:
            self.pub_cmd.publish(Twist())                  # paro total

        elif self.state == StateFSM.REDUCE_VEL:
            # Misma ley de control pero velocidades y kp a la mitad
            self.move_robot(self.v_max/2, self.v_min/2, self.kp_ang/2)

        elif self.state == StateFSM.FOLLOW_LINE:
            self.move_robot(self.v_max, self.v_min, self.kp_ang)

        # Una vez enviada la acción, se espera una nueva bandera externa
        self.state = None


def main(args=None):
    rclpy.init(args=args)
    node = CmdRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
