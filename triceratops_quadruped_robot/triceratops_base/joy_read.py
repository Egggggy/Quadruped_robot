import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyReader(Node):
    def __init__(self):
        super().__init__('joy_reader')
        self.create_subscription(Joy, 'joy', self.cb, 1)
        self.prev_buttons = []
        self.prev_axes = []

    def cb(self, msg):
        if msg.buttons != self.prev_buttons or [round(a, 2) for a in msg.axes] != [round(a, 2) for a in self.prev_axes]:
            print(f"buttons: {list(msg.buttons)}")
            print(f"axes:    {[round(a, 2) for a in msg.axes]}")
            print()
        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)

rclpy.init()
rclpy.spin(JoyReader())