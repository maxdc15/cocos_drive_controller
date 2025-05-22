# Nodo ROS 2 que implementa una máquina de estados para gestionar
# la velocidad del robot según el color detectado en un semáforo.
# Publica tres flags booleanos:
#   /follow_line      → velocidad normal
#   /reduce_velocity  → velocidad reducida (amarillo)
#   /stop             → detenerse (rojo)
#
# Transiciones:
#   • FOLLOW_LINE ── amarillo ──► REDUCE_VEL
#   • FOLLOW_LINE ── rojo ──────► STOP
#   • REDUCE_VEL  ── rojo ──────► STOP
#   • STOP        ── verde ─────► FOLLOW_LINE
#   (cualquier otro caso conserva el estado actual)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from enum import Enum, auto


# ---------- Enumeración de estados de la FSM ----------
class StateFSM(Enum):
    FOLLOW_LINE = auto()
    REDUCE_VEL  = auto()
    STOP        = auto()


class MainController(Node):
    """Máquina de estados: línea, reducir velocidad y paro."""

    # ------------------------------------------------------------------ #
    def __init__(self):
        super().__init__("main_controller")
        self.get_logger().info("Nodo main_controller iniciado")

        # ── Publicadores ------------------------------------------------
        self.pub_follow_line = self.create_publisher(Bool, '/follow_line',      1)
        self.pub_reduce      = self.create_publisher(Bool, '/reduce_velocity',  1)
        self.pub_stop        = self.create_publisher(Bool, '/stop',             1)

        # ── Suscriptores ------------------------------------------------
        self.create_subscription(Bool, '/detected_color', self.detected_cb, 10)
        self.create_subscription(Int8, '/color',          self.color_cb,    10)

        # ── Variables internas -----------------------------------------
        self.detected_color = False   # True si ve semáforo
        self.color          = 0       # 1-rojo, 2-amarillo, 3-verde
        self.state          = StateFSM.FOLLOW_LINE

        # Mensajes pre-alocados (evita crear objetos cada ciclo)
        self.msg_f = Bool()
        self.msg_r = Bool()
        self.msg_s = Bool()

        # Loop principal a 10 Hz
        self.timer = self.create_timer(0.1, self.loop)

    # ------------------------------------------------------------------ #
    # Callbacks de suscriptores
    def detected_cb(self, msg: Bool):
        self.detected_color = msg.data

    def color_cb(self, msg: Int8):
        self.color = msg.data

    # ------------------------------------------------------------------ #
    # Lógica de la máquina de estados
    def loop(self):
        # 1) Estado STOP: sólo sale con verde
        if self.state == StateFSM.STOP:
            if self.detected_color and self.color == 3:
                self.state = StateFSM.FOLLOW_LINE

        # 2) Estado REDUCE_VEL: permanece hasta ver rojo (pasa a STOP)
        elif self.state == StateFSM.REDUCE_VEL:
            if self.detected_color and self.color == 1:
                self.state = StateFSM.STOP
            # Verde o ausencia de señal NO cambian este estado

        # 3) Estado FOLLOW_LINE: reacciona a rojo o amarillo
        else:  # self.state == FOLLOW_LINE
            if self.detected_color:
                if   self.color == 1:  # rojo
                    self.state = StateFSM.STOP
                elif self.color == 2:  # amarillo
                    self.state = StateFSM.REDUCE_VEL
                # verde u otros colores: sigue igual

        # ---------- Publicar flags ----------
        self.msg_f.data = (self.state == StateFSM.FOLLOW_LINE)
        self.msg_r.data = (self.state == StateFSM.REDUCE_VEL)
        self.msg_s.data = (self.state == StateFSM.STOP)

        self.pub_follow_line.publish(self.msg_f)
        self.pub_reduce.publish(self.msg_r)
        self.pub_stop.publish(self.msg_s)


# ================================ main ===================================== #
def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
