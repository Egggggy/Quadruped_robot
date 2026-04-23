import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

BUTTON_A     = 0
BUTTON_B     = 1
BUTTON_X     = 3
BUTTON_Y     = 4
BUTTON_LB    = 6
BUTTON_RB    = 7
BUTTON_BACK  = 10
BUTTON_START = 11


class TriceratopsControlClient(Node):
    def __init__(self):
        super().__init__('JoyControlClient')
        self.create_subscription(Joy, "joy", self.joy_callback, 1)
        self.cmd_pub  = self.create_publisher(Twist,  'cmd_vel',      1)
        self.mode_pub = self.create_publisher(String, '/robot_mode',  1)

        self.prev_buttons = []

        self.linear_x_scale = 0.1
        self.linear_y_scale = 0.08
        self.angular_scale  = 1
        self.joy = None

        self.timer = self.create_timer(1/500, self.timer_callback)
        self.get_logger().info("Client Initialized")

    def _rising_edge(self, data, index):
        if len(self.prev_buttons) <= index:
            return False
        return data.buttons[index] == 1 and self.prev_buttons[index] == 0

    def _publish_mode(self, mode):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"[mode] {mode}")

    def joy_callback(self, data: Joy):
        lb = data.buttons[BUTTON_LB] == 1

        # LB + RB + X → stop（三鍵安全設計，最優先）
        if lb and data.buttons[BUTTON_RB] == 1 and data.buttons[BUTTON_X] == 1:
            self._publish_mode("stop")

        # LB 組合鍵（頭部控制）
        elif lb and self._rising_edge(data, BUTTON_A):
            self._publish_mode("shake_head_left")
        elif lb and self._rising_edge(data, BUTTON_B):
            self._publish_mode("shake_head_right")
        elif lb and self._rising_edge(data, BUTTON_Y):
            self._publish_mode("shake_head_center")

        # 單鍵
        elif self._rising_edge(data, BUTTON_START):
            self._publish_mode("start_gait")
        elif self._rising_edge(data, BUTTON_A):
            self._publish_mode("reset")
        elif self._rising_edge(data, BUTTON_Y):
            self._publish_mode("handshake")
        elif self._rising_edge(data, BUTTON_X):
            self._publish_mode("sit")

        self.prev_buttons = list(data.buttons)
        self.joy = data

    def timer_callback(self):
        vel = Twist()
        if self.joy is not None:
            vel.linear.x  = self.joy.axes[1] * self.linear_x_scale
            vel.linear.y  = self.joy.axes[0] * self.linear_y_scale
            vel.angular.z = self.joy.axes[2] * self.angular_scale
        self.cmd_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    controller = TriceratopsControlClient()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
