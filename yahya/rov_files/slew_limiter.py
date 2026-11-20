#!/usr/bin/env python3
"""slew_limiter: soft-start, the LAST stage before the ESCs.

In : /thruster_cmd (Float32MultiArray, -1..1), /armed (Bool)
Out: rov_power (Int16MultiArray, -1000..1000)   <- same topic your display already reads
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int16MultiArray


class SlewLimiter(Node):
    def __init__(self):
        super().__init__('slew_limiter')
        self.declare_parameter('max_rate', 2.0)        # full-scale units per second
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('cmd_timeout', 0.3)     # s: stale command -> zero
        self.declare_parameter('max_power', 1000)
        self.declare_parameter('invert', [1, 1, 1, 1, 1, 1])  # -1 flips a wired-backwards thruster

        self.target = np.zeros(6)
        self.out = np.zeros(6)
        self.armed = False
        self.last_cmd = 0.0
        self.last_armed = 0.0

        self.create_subscription(Float32MultiArray, 'thruster_cmd', self.on_cmd, 10)
        self.create_subscription(Bool, 'armed', self.on_armed, 10)
        self.pub = self.create_publisher(Int16MultiArray, 'rov_power', 10)
        self.dt = 1.0 / self.get_parameter('rate_hz').value
        self.create_timer(self.dt, self.tick)

    def on_cmd(self, msg):
        if len(msg.data) == 6:
            self.target = np.array(msg.data, dtype=float)
            self.last_cmd = time.monotonic()

    def on_armed(self, msg):
        self.armed = msg.data
        self.last_armed = time.monotonic()

    def tick(self):
        now = time.monotonic()
        tmo = self.get_parameter('cmd_timeout').value
        safe_to_drive = self.armed and (now - self.last_armed < tmo) and (now - self.last_cmd < tmo)

        if not safe_to_drive:
            self.out[:] = 0.0                      # ZERO-BYPASS: cut immediately, no ramp-down
        else:
            step = self.get_parameter('max_rate').value * self.dt
            self.out += np.clip(self.target - self.out, -step, step)

        inv = np.array(self.get_parameter('invert').value, dtype=float)
        scale = self.get_parameter('max_power').value
        self.pub.publish(Int16MultiArray(data=[int(round(x)) for x in self.out * inv * scale]))


def main():
    rclpy.init()
    node = SlewLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
