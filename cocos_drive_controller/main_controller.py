import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from enum import Enum, auto

class StateFSM(Enum):
    FOLLOW_LINE = auto()
    REDUCE_VEL  = auto()
    STOP        = auto()

class MainController(Node):

    def __init__(self):
        super().__init__("main_controller")
        self.get_logger().info("Nodo main_controller iniciado")

        # ── Publishers ───────────────────────────────────────────

        self.pub_follow_line = self.create_publisher(Bool, '/follow_line', 1)
        self.pub_stop        = self.create_publisher(Bool, '/stop',        1)
        self.pub_reduce      = self.create_publisher(Bool, '/reduce_velocity', 1)

        # ── Subscribers ──────────────────────────────────────────

        self.create_subscription(Bool, '/detected_color', self.detected_color_callback, 10)
        self.create_subscription(Int8, '/color', self.color_callback, 10)

        # ── Variables ───────────────────────────────────────────

        self.detected_color = False
        self.color = 0
        self.state          = StateFSM.FOLLOW_LINE


        self.timer = self.create_timer(0.1, self.loop)   # 10 Hz


    def detected_color_callback(self, msg):
        self.detected_color = msg.data


    def color_callback(self, msg):
        self.color = msg.data

    def loop(self):
        # Si estamos en STOP, solo salimos cuando venga verde
        if self.state == StateFSM.STOP:
            if self.detected_color and self.color == 3:
                self.state = StateFSM.FOLLOW_LINE
            # si no, quedamos en STOP

        else:
            # Si detectamos señal, evaluamos color
            if self.detected_color:
                if self.color == 1:
                    self.state = StateFSM.STOP
                elif self.color == 2:
                    self.state = StateFSM.REDUCE_VEL
                elif self.color == 3:
                    self.state = StateFSM.FOLLOW_LINE
                else:
                    self.state = StateFSM.FOLLOW_LINE
            else:
                # sin señal → sigue línea
                self.state = StateFSM.FOLLOW_LINE

        # Publicar los tres flags (solo uno true)
        msg_f = Bool(); msg_r = Bool(); msg_s = Bool()
        msg_f.data = (self.state == StateFSM.FOLLOW_LINE)
        msg_r.data = (self.state == StateFSM.REDUCE_VEL)
        msg_s.data = (self.state == StateFSM.STOP)

        self.pub_follow_line.publish(msg_f)
        self.pub_reduce.publish(msg_r)
        self.pub_stop.publish(msg_s)


def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        

        