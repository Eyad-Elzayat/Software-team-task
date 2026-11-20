# ROS 2 package: thruster_ctl   (your display, plus the current mode)
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String


class Display(Node):
    def __init__(self):
        super().__init__('thruster_display')
        self.direction = 'STOP'
        self.mode = '?'
        self.last = self.get_clock().now()
        self.create_subscription(String, 'rov_direction', self.on_dir, 10)
        self.create_subscription(String, 'mode', self.on_mode, 10)
        self.create_subscription(Int16MultiArray, 'rov_power', self.on_power, 10)
        self.create_timer(0.5, self.watchdog)

    def show(self, text):
        sys.stdout.write('\r' + text + '\033[K')  # overwrite the same line
        sys.stdout.flush()

    def on_dir(self, msg):
        self.direction = msg.data

    def on_mode(self, msg):
        self.mode = msg.data

    def on_power(self, msg):
        self.last = self.get_clock().now()
        m = msg.data
        self.show('[%s] m1=%5d  m2=%5d  m3=%5d  m4=%5d  m5=%5d  m6=%5d  |  %s'
                  % (self.mode, *m, self.direction))

    def watchdog(self):
        if (self.get_clock().now() - self.last).nanoseconds > 2e9:
            self.show('No data from the slew limiter')


def main():
    rclpy.init()
    node = Display()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
