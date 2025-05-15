import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from rclpy.clock import Clock
from rclpy.duration import Duration
from enum import Enum, auto

class StateFSM(Enum):
    FOLLOW_LINE = auto()
    STOP = auto()
    REDUCE_VEL = auto() 


class CmdRobotNode(Node):
    def __init__(self):
        super().__init__('cmd_robot_node')
        self.get_logger().info('Nodo cmd_robot_node iniciado')

        # -- parameters -- 

        self.declare_parameter('v_max',     0.18)   # velocidad recto   [m/s]
        self.declare_parameter('v_min',     0.05)   # mínima en curva   [m/s]
        self.declare_parameter('kp_ang',    0.6)    # ganancia angular  [rad/s]
        self.declare_parameter('threshold', 0.05)   # error bajo el cual se va recto
        self.declare_parameter('timeout',   0.5)    # detiene si no hay datos [s]
        self.declare_parameter('rate', 50.0) 

        # -- get parameters --

        self.v_max    = self.get_parameter('v_max').value
        self.v_min    = self.get_parameter('v_min').value
        self.kp_ang   = self.get_parameter('kp_ang').value
        self.thresh   = self.get_parameter('threshold').value
        self.timeout  = self.get_parameter('timeout').value
        self.dt = 1.0 / self.get_parameter('rate').value

        # -- publishers

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 1)

        # -- subscribers --

        self.create_subscription(Float32, '/line_error', self.callback_line_error, 10)
        self.create_subscription(Bool, '/stop', self.stop_robot, 10)
        self.create_subscription(Bool, '/reduce_velocity', self.reduce_velocity, 10)
        self.create_subscription(Bool, '/follow_line', self.move_robot, 10)

        # -- variables --

        self.error_line = None

        self.stop_state = False
        self.reduce_state = False
        self.follow_state = False

        self.last_msg_time = Clock().now()

        # -- timer --

        self.timer = self.create_timer(self.dt, self.control_loop)

    def callback_line_error(self, msg):
        self.error_line = msg.data
        self.last_msg_time = Clock().now()

    def stop_robot(self, msg):
        self.stop_state = msg.data

    def reduce_velocity(self, msg):
        self.reduce_state = msg.data

    def move_robot(self, msg):
        self.follow_state = msg.data


    def move_robot(self, vmax, vmin, kp_ang):

        twist = Twist()
        abs_error = abs(self.error_line)

        if abs_error <= self.thresh:

            twist.linear.x = vmax
            twist.angular.z = 0.0
        else:
            twist.angular.z = - kp_ang * self.error_line
        
            scale = (abs_error - self.thresh) / (1.0 - self.thresh)
            twist.linear.x = max(vmin, vmax * (1.0 - scale))

        self.pub_cmd.publish(twist)

    def stop_robot(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.pub_cmd.publish(stop)

    
    def reduce_velocity(self):
        self.move_robot(self.v_max/2, self.v_min/2, self.kp_ang/2)


    def current_signal(self):
        if self.stop_state:
            self.stop_state = False
            return StateFSM.STOP
        
        if self.reduce_state:
            self.reduce_state = False
            return StateFSM.REDUCE_VEL
        
        if self.follow_state:
            self.follow_state = False
            return StateFSM.FOLLOW_LINE
        
        return None



    def control_loop(self):
        if None in (self.error_line):
            return
        
        event = self.current_signal()

        self.watchdog()

        if event is not None:
            self.state = event

        if event == StateFSM.STOP:
            self.stop_robot()
        elif event == StateFSM.REDUCE_VEL:
            self.reduce_velocity()
        elif event == StateFSM.FOLLOW_LINE:
            self.move_robot(self.v_max, self.v_min, self.kp_ang)
        
    
    def watchdog(self):
        if (Clock().now() - self.last_msg_time) > Duration(seconds=self.timeout):
            stop = Twist()
            self.pub_cmd.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = CmdRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


