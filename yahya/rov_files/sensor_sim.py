"""Closed-loop sensor simulator (replaces the random one).
Reads the REAL final thruster outputs (rov_power) and integrates a tiny vehicle model,
so depth hold and heading hold have something to control on the bench.
Publishes the same topics/types as before: imu [roll,pitch,yaw] deg, depth metres (+down).
"""
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray


class SensorSim(Node):
    def __init__(self):
        super().__init__('sensor_sim')
        self.declare_parameter('start_depth', 1.0)
        self.declare_parameter('max_vertical_speed', 0.5)   # m/s at full heave
        self.declare_parameter('max_yaw_rate', 60.0)        # deg/s at full yaw
        self.depth = self.get_parameter('start_depth').value
        self.vz = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.heave_cmd = 0.0
        self.yaw_cmd = 0.0
        self.dt = 0.05

        self.create_subscription(Int16MultiArray, 'rov_power', self.on_power, 10)
        self.imu_pub = self.create_publisher(Float32MultiArray, 'imu', 10)
        self.depth_pub = self.create_publisher(Float32, 'depth', 10)
        self.create_timer(self.dt, self.tick)       # 20 Hz

    def on_power(self, msg):
        m = msg.data
        if len(m) == 6:
            self.heave_cmd = (m[4] + m[5]) / 2000.0                 # + = up
            self.yaw_cmd = (m[0] - m[1] + m[2] - m[3]) / 4000.0     # + = CCW

    def tick(self):
        vz_target = -self.get_parameter('max_vertical_speed').value * self.heave_cmd
        self.vz += (vz_target - self.vz) * self.dt / 0.5            # first-order lag
        self.depth = max(0.0, self.depth + self.vz * self.dt)

        yr_target = self.get_parameter('max_yaw_rate').value * self.yaw_cmd
        self.yaw_rate += (yr_target - self.yaw_rate) * self.dt / 0.3
        self.yaw = (self.yaw + self.yaw_rate * self.dt) % 360.0

        self.imu_pub.publish(Float32MultiArray(data=[
            random.gauss(0, 0.1), random.gauss(0, 0.1), self.yaw + random.gauss(0, 0.1)]))
        self.depth_pub.publish(Float32(data=float(self.depth + random.gauss(0, 0.01))))


def main():
    rclpy.init()
    node = SensorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
