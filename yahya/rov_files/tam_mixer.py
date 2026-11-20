#!/usr/bin/env python3
"""tam_mixer: Thruster Allocation Matrix mixer.

Input : /cmd_wrench (Wrench, each axis -1..1)
Output: /thruster_cmd (Float32MultiArray, 6 values m1..m6, each -1..1)

Frame (REP-103): x forward, y left, z up. Positions in metres from the centre.
Thrusters: 1 front-right, 2 front-left, 3 back-right, 4 back-left (angled 45 deg),
           5 right vertical, 6 left vertical.
EDIT the positions/directions below to match your real frame.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float32MultiArray, String

S = np.sqrt(0.5)
L = 0.15   # half-distance of thrusters from the centre (m)

# (position r, direction d of POSITIVE thrust)
THRUSTERS = [
    (np.array([+L, -L, 0.0]), np.array([+S, +S, 0.0])),   # 1 front-right
    (np.array([+L, +L, 0.0]), np.array([+S, -S, 0.0])),   # 2 front-left
    (np.array([-L, -L, 0.0]), np.array([+S, -S, 0.0])),   # 3 back-right
    (np.array([-L, +L, 0.0]), np.array([+S, +S, 0.0])),   # 4 back-left
    (np.array([0.0, -L, 0.0]), np.array([0.0, 0.0, +1.0])),  # 5 right vertical
    (np.array([0.0, +L, 0.0]), np.array([0.0, 0.0, +1.0])),  # 6 left vertical
]


def build_B():
    """6x6 matrix. Column i = [force direction ; torque = r x d] of thruster i."""
    cols = [np.hstack([d, np.cross(r, d)]) for r, d in THRUSTERS]
    return np.column_stack(cols)


def direction(w, db=0.1):
    d = []
    if abs(w.force.x) > db: d.append('FORWARD' if w.force.x > 0 else 'BACKWARD')
    if abs(w.force.y) > db: d.append('LEFT' if w.force.y > 0 else 'RIGHT')
    if abs(w.torque.z) > db: d.append('TURN LEFT' if w.torque.z > 0 else 'TURN RIGHT')
    if abs(w.force.z) > db: d.append('UP' if w.force.z > 0 else 'DOWN')
    return ' + '.join(d) if d else 'STOP'


class TamMixer(Node):
    def __init__(self):
        super().__init__('tam_mixer')
        B = build_B()
        self.Bp = np.linalg.pinv(B)           # computed ONCE, not every message

        # Axis normalisation: make "full stick on one axis" = strongest thruster at 1.0.
        # Without this, full surge would only give ~0.35 output.
        col_max = np.max(np.abs(self.Bp), axis=0)
        self.gain = np.where(col_max > 1e-9, 1.0 / np.maximum(col_max, 1e-9), 0.0)

        self.get_logger().info(
            f'B rank = {np.linalg.matrix_rank(B)} (6 = all DOF controllable)')
        self.get_logger().info(f'axis gains [Fx Fy Fz Mx My Mz] = {np.round(self.gain, 2)}')

        self.create_subscription(Wrench, '/cmd_wrench', self.on_wrench, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/thruster_cmd', 10)
        self.dir_pub = self.create_publisher(String, 'rov_direction', 10)

    def on_wrench(self, msg):
        tau = np.array([msg.force.x, msg.force.y, msg.force.z,
                        msg.torque.x, msg.torque.y, msg.torque.z]) * self.gain
        u = self.Bp @ tau

        peak = np.max(np.abs(u))
        if peak > 1.0:
            u = u / peak                      # proportional desaturation, keeps direction

        self.pub.publish(Float32MultiArray(data=[float(x) for x in u]))
        self.dir_pub.publish(String(data=direction(msg)))


def main():
    rclpy.init()
    node = TamMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
