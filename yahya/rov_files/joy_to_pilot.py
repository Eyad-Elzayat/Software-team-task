#!/usr/bin/env python3
"""joy_to_pilot: /joy (from udp_bridge)  ->  /pilot_cmd (Twist, -1..1) + button -> service calls.

Conversion to the REP-103 convention used by mode_manager and tam_mixer
(x forward, y LEFT, z up, yaw CCW positive):
  surge =  -axis1 (stick up = -1 in pygame)     sway = -axis0 (stick right = right = -y)
  yaw   =  -axis2 (stick right = clockwise = -) heave = -axis3 (stick up = up)
Xbox 360 via pygame buttons: A=0 B=1 X=2 Y=3 LB=4 RB=5 Back=6 Start=7
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

BUTTON_SERVICES = {          # parameter name -> service
    'btn_arm': '/arm',
    'btn_disarm': '/disarm',
    'btn_manual': '/mode/manual',
    'btn_depth_hold': '/mode/depth_hold',
    'btn_heading_hold': '/mode/heading_hold',
}


class JoyToPilot(Node):
    def __init__(self):
        super().__init__('joy_to_pilot')
        self.declare_parameter('deadzone', 0.08)
        for name, idx in (('axis_surge', 1), ('axis_sway', 0), ('axis_yaw', 2), ('axis_heave', 3)):
            self.declare_parameter(name, idx)
        for name, idx in (('btn_arm', 7), ('btn_disarm', 6), ('btn_manual', 0),
                          ('btn_depth_hold', 2), ('btn_heading_hold', 3)):
            self.declare_parameter(name, idx)

        self.pub = self.create_publisher(Twist, 'pilot_cmd', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.btn_clients = {k: self.create_client(Trigger, v) for k, v in BUTTON_SERVICES.items()}
        self.prev = {k: 0 for k in BUTTON_SERVICES}

    def axis(self, axes, param):
        i = self.get_parameter(param).value
        v = axes[i] if 0 <= i < len(axes) else 0.0
        dz = self.get_parameter('deadzone').value
        return 0.0 if abs(v) < dz else max(-1.0, min(1.0, v))

    def on_joy(self, msg):
        t = Twist()
        t.linear.x = -self.axis(msg.axes, 'axis_surge')
        t.linear.y = -self.axis(msg.axes, 'axis_sway')
        t.linear.z = -self.axis(msg.axes, 'axis_heave')
        t.angular.z = -self.axis(msg.axes, 'axis_yaw')
        self.pub.publish(t)          # every packet = one heartbeat for the watchdog

        for key, cli in self.btn_clients.items():
            i = self.get_parameter(key).value
            pressed = msg.buttons[i] if 0 <= i < len(msg.buttons) else 0
            if pressed and not self.prev[key]:          # rising edge only
                self.call(key, cli)
            self.prev[key] = pressed

    def call(self, key, cli):
        if not cli.service_is_ready():
            self.get_logger().warn(f'{key}: service not available (is mode_manager running?)')
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f, k=key: self.get_logger().info(
            f'{k}: {f.result().message}' if f.result() else f'{k}: call failed'))


def main():
    rclpy.init()
    node = JoyToPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
