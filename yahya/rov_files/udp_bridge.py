"""UDP bridge: the ONLY place where the non-ROS world (Windows topside) touches ROS 2.

Topside -> here : JSON {"axes": [...], "buttons": [...]}  -> published as sensor_msgs/Joy on /joy
Here -> topside : JSON {"roll","pitch","yaw","depth"}     <- built from /imu and /depth
Replies go to whatever address the last joystick packet came from, so we never
need to know the Windows IP and the Windows firewall lets replies through.
If joystick packets stop, we publish a neutral /joy (simple loss-of-comms failsafe).
"""
import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float32MultiArray


class UdpBridge(Node):
    def __init__(self):
        super().__init__('udp_bridge')
        self.declare_parameter('port', 5005)
        self.declare_parameter('telemetry_hz', 10.0)
        self.declare_parameter('timeout_s', 1.0)
        # Neutral (safe) value of each axis. Xbox 360 via pygame: sticks rest at 0,
        # triggers (axes 4 and 5) rest at -1.0.
        self.declare_parameter('neutral_axes', [0.0, 0.0, 0.0, 0.0, -1.0, -1.0])
        self.declare_parameter('publish_neutral_on_timeout', False)

        port = self.get_parameter('port').value
        self.timeout_s = self.get_parameter('timeout_s').value
        self.neutral_axes = list(self.get_parameter('neutral_axes').value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.2)

        self.topside = None          # (ip, port) of the topside program
        self.last_rx = 0.0           # time of last joystick packet
        self.n_axes, self.n_buttons = 6, 12
        self.imu = [0.0, 0.0, 0.0]   # roll, pitch, yaw
        self.depth = 0.0

        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.create_subscription(Float32MultiArray, 'imu', self.on_imu, 10)
        self.create_subscription(Float32, 'depth', self.on_depth, 10)
        self.create_timer(1.0 / self.get_parameter('telemetry_hz').value, self.send_telemetry)
        self.create_timer(0.1, self.watchdog)

        self.running = True
        threading.Thread(target=self.rx_loop, daemon=True).start()
        self.get_logger().info(f'Listening for topside joystick on UDP :{port}')

    # ---- topside -> ROS ----
    def rx_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
                axes = [float(a) for a in msg['axes']]
                buttons = [int(b) for b in msg.get('buttons', [])]
            except (ValueError, KeyError, TypeError):
                continue  # ignore garbage packets
            if self.topside != addr:
                self.get_logger().info(f'Topside connected from {addr}')
            self.topside = addr
            self.last_rx = time.monotonic()
            self.n_axes, self.n_buttons = len(axes), len(buttons)
            self.publish_joy(axes, buttons)

    def publish_joy(self, axes, buttons):
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = axes
        joy.buttons = buttons
        self.joy_pub.publish(joy)

    def watchdog(self):
        if self.topside is None:
            return
        if time.monotonic() - self.last_rx > self.timeout_s:
            self.get_logger().warn('No joystick packets from topside',
                                   throttle_duration_sec=2.0)
            if not self.get_parameter('publish_neutral_on_timeout').value:
                return  # silence on purpose: mode_manager's watchdog sees it and DISARMS
            neutral = [self.neutral_axes[i] if i < len(self.neutral_axes) else 0.0
                       for i in range(self.n_axes)]
            self.publish_joy(neutral, [0] * self.n_buttons)

    # ---- ROS -> topside ----
    def on_imu(self, msg):
        self.imu = list(msg.data[:3])

    def on_depth(self, msg):
        self.depth = msg.data

    def send_telemetry(self):
        if self.topside is None:
            return
        pkt = {'roll': self.imu[0], 'pitch': self.imu[1], 'yaw': self.imu[2],
               'depth': self.depth}
        try:
            self.sock.sendto(json.dumps(pkt).encode(), self.topside)
        except OSError:
            pass

    def destroy_node(self):
        self.running = False
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = UdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
