import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

BUTTON_Y     = 3
BUTTON_B     = 1
BUTTON_A     = 0
BUTTON_START = 9
BUTTON_LB    = 4
BUTTON_RB    = 5
BUTTON_X     = 2
# from champ_interfaces.srv import SetMode


class TriceratopsControlClient(Node):
    def __init__(self):
        super().__init__('JoyControlClient')
        self.create_subscription(Joy, "joy", self.joy_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 1)

        self.current_mode = ""
        self.prev_buttons = []

        self.linear_x_scale = 0.1
        self.linear_y_scale = 0.08
        self.angular_scale = 1
        feq = 500
        self.joy = None
        self.cmd = None
        
        self.timer = self.create_timer(1/feq, self.timer_callback)
        self.get_logger().info("Client Initialized")
        
    def _rising_edge(self, data, index):
        """偵測按鍵由放開到按下的瞬間（避免長按重複觸發）"""
        if len(self.prev_buttons) <= index:
            return False
        return data.buttons[index] == 1 and self.prev_buttons[index] == 0

    def _publish_mode(self, mode):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"[mode] {mode}")

    def joy_callback(self, data: Joy):
        # 組合鍵：LB + RB + X → stop
        if data.buttons[BUTTON_LB] == 1 and data.buttons[BUTTON_RB] == 1 and data.buttons[BUTTON_X] == 1:
            self.current_mode = "stop"
            self._publish_mode("stop")

        elif self._rising_edge(data, BUTTON_Y):
            self.current_mode = "handshake"
            self._publish_mode("handshake")

        elif self._rising_edge(data, BUTTON_B):
            self.current_mode = "sway"
            self._publish_mode("sway")

        elif self._rising_edge(data, BUTTON_START):
            self.current_mode = "start_gait"
            self._publish_mode("start_gait")

        elif self._rising_edge(data, BUTTON_A):
            self.current_mode = "reset"
            self._publish_mode("reset")

        elif data.axes[3] == 1 and self.current_mode != "up":
            self.current_mode = "up"
        elif data.axes[3] == -1 and self.current_mode != "down":
            self.current_mode = "down"
        elif data.axes[2] == 1 and self.current_mode != "left":
            self.current_mode = "left"
        elif data.axes[2] == -1 and self.current_mode != "right":
            self.current_mode = "right"
        else:
            self.current_mode = " "

        self.prev_buttons = list(data.buttons)
        self.joy = data

    def timer_callback(self):
        vel = Twist()
        
        if self.joy is not None:
            if self.joy.axes[1]>0.001:
                vel.linear.x = self.joy.axes[1]*self.linear_x_scale
            elif self.joy.axes[1]<-0.001:
                vel.linear.x = self.joy.axes[1]*self.linear_x_scale
            if self.joy.axes[0]>0.001:
                vel.linear.y = self.joy.axes[0]*self.linear_y_scale
            elif self.joy.axes[0]<-0.001:
                vel.linear.y = self.joy.axes[0]*self.linear_y_scale
            if self.joy.axes[2]>0.001:
                vel.angular.z = self.joy.axes[2]*self.angular_scale
            elif self.joy.axes[2]<-0.001:
                vel.angular.z = self.joy.axes[2]*self.angular_scale

        self.cmd_pub.publish(vel)
        
def main(args=None):
    rclpy.init(args=args)

    controller = TriceratopsControlClient()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
