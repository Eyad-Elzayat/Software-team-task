#!/usr/bin/env python3
"""mode_manager: the ONLY node that publishes /cmd_wrench.

Inputs : /pilot_cmd (Twist, -1..1), /imu (Float32MultiArray [roll,pitch,yaw] deg), /depth (Float32, m, +down)
Outputs: /cmd_wrench (Wrench), /mode (String), /armed (Bool)
Services (std_srvs/Trigger): /arm /disarm /mode/manual /mode/depth_hold /mode/heading_hold

Wrench convention (ROS REP-103): x forward, y left, z up.
  force.x = surge, force.y = sway, force.z = heave (+up), torque.z = yaw (+CCW)
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from std_srvs.srv import Trigger

DISARMED, MANUAL, DEPTH_HOLD, HEADING_HOLD = (
    'DISARMED', 'MANUAL', 'DEPTH_HOLD', 'HEADING_HOLD')


def clamp(x, lim):
    return max(-lim, min(lim, x))


def wrap_pi(a):
    """Wrap angle to [-pi, pi] so heading error never takes the long way round."""
    return math.atan2(math.sin(a), math.cos(a))


class PID:
    def __init__(self):
        self.kp = self.ki = self.kd = 0.0
        self.i_limit = 0.5      # max size of the integral TERM (anti-windup)
        self.out_limit = 1.0
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev_err = None

    def update(self, err, dt):
        self.i = clamp(self.i + self.ki * err * dt, self.i_limit)
        d = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        self.prev_err = err
        return clamp(self.kp * err + self.i + self.kd * d, self.out_limit)


class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')

        # ---- parameters (change live with: ros2 param set /mode_manager depth_kp 2.0)
        defaults = {
            'rate_hz': 50.0,
            'link_timeout': 0.4,      # s without pilot packet -> DISARM
            'sensor_timeout': 0.5,    # s without IMU/depth in a hold mode -> MANUAL
            'stick_deadband': 0.1,
            'depth_kp': 1.5, 'depth_ki': 0.2, 'depth_kd': 0.3,
            'heading_kp': 0.8, 'heading_ki': 0.05, 'heading_kd': 0.2,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)

        # ---- state
        self.mode = DISARMED
        self.pilot = Twist()
        self.last_pilot = 0.0           # monotonic time of last valid packet
        self.depth = 0.0
        self.yaw = 0.0
        self.last_depth = 0.0
        self.last_imu = 0.0
        self.depth_sp = 0.0
        self.yaw_sp = 0.0
        self.depth_pid = PID()
        self.heading_pid = PID()

        # ---- ROS interfaces
        self.create_subscription(Twist, '/pilot_cmd', self.on_pilot, 10)
        self.create_subscription(Float32MultiArray, '/imu', self.on_imu, 10)
        self.create_subscription(Float32, '/depth', self.on_depth, 10)
        self.wrench_pub = self.create_publisher(Wrench, '/cmd_wrench', 10)
        self.mode_pub = self.create_publisher(String, '/mode', 10)
        self.armed_pub = self.create_publisher(Bool, '/armed', 10)

        self.create_service(Trigger, '/arm', self.srv(MANUAL))
        self.create_service(Trigger, '/disarm', self.srv(DISARMED))
        self.create_service(Trigger, '/mode/manual', self.srv(MANUAL))
        self.create_service(Trigger, '/mode/depth_hold', self.srv(DEPTH_HOLD))
        self.create_service(Trigger, '/mode/heading_hold', self.srv(HEADING_HOLD))

        period = 1.0 / self.get_parameter('rate_hz').value
        self.dt = period
        self.create_timer(period, self.tick)
        self.get_logger().info('mode_manager ready, state DISARMED')

    # ------------------------------------------------------------ callbacks
    def on_pilot(self, msg):
        self.pilot = msg
        self.last_pilot = time.monotonic()      # this is the heartbeat

    def on_imu(self, msg):
        self.yaw = math.radians(msg.data[2])    # degrees -> radians
        self.last_imu = time.monotonic()

    def on_depth(self, msg):
        self.depth = msg.data
        self.last_depth = time.monotonic()

    def srv(self, target):
        def cb(_req, res):
            res.success, res.message = self.request_mode(target)
            return res
        return cb

    # ------------------------------------------------------- helper checks
    def link_ok(self):
        return time.monotonic() - self.last_pilot < self.get_parameter('link_timeout').value

    def sensor_ok(self, last):
        return time.monotonic() - last < self.get_parameter('sensor_timeout').value

    def sticks_centered(self):
        db = self.get_parameter('stick_deadband').value
        p = self.pilot
        return all(abs(v) < db for v in
                   (p.linear.x, p.linear.y, p.linear.z, p.angular.z))

    # ---------------------------------------------------------- mode logic
    def request_mode(self, target):
        if target == DISARMED:
            self.set_mode(DISARMED, 'disarm requested')
            return True, 'DISARMED'

        # pre-arm checks (improvement #1): only when coming from DISARMED
        if self.mode == DISARMED:
            if target != MANUAL:
                return False, 'arm first (/arm), then pick a mode'
            if not self.link_ok():
                return False, 'pre-arm failed: no fresh pilot packets'
            if not self.sticks_centered():
                return False, 'pre-arm failed: centre the sticks'

        if target == DEPTH_HOLD and not self.sensor_ok(self.last_depth):
            return False, 'depth sensor stale'
        if target == HEADING_HOLD and not self.sensor_ok(self.last_imu):
            return False, 'IMU stale'

        self.set_mode(target, 'requested')
        return True, target

    def set_mode(self, new_mode, why):
        if new_mode == self.mode:
            return
        # clean switch: reset integrators and latch setpoints = no windup, no jump
        self.depth_pid.reset()
        self.heading_pid.reset()
        self.depth_sp = self.depth
        self.yaw_sp = self.yaw
        self.get_logger().warn(f'{self.mode} -> {new_mode} ({why})')
        self.mode = new_mode

    # ---------------------------------------------------------------- loop
    def tick(self):
        # --- failsafes first, before anything else
        if self.mode != DISARMED and not self.link_ok():
            self.set_mode(DISARMED, 'WATCHDOG: topside link lost')
        if self.mode == DEPTH_HOLD and not self.sensor_ok(self.last_depth):
            self.set_mode(MANUAL, 'depth sensor stale')
        if self.mode == HEADING_HOLD and not self.sensor_ok(self.last_imu):
            self.set_mode(MANUAL, 'IMU stale')

        # --- update PID gains from parameters (enables live tuning)
        for pid, name in ((self.depth_pid, 'depth'), (self.heading_pid, 'heading')):
            pid.kp = self.get_parameter(f'{name}_kp').value
            pid.ki = self.get_parameter(f'{name}_ki').value
            pid.kd = self.get_parameter(f'{name}_kd').value

        w = Wrench()
        if self.mode != DISARMED:
            p = self.pilot
            db = self.get_parameter('stick_deadband').value
            w.force.x = p.linear.x
            w.force.y = p.linear.y
            w.force.z = p.linear.z
            w.torque.z = p.angular.z

            if self.mode == DEPTH_HOLD:
                if abs(p.linear.z) > db:
                    # pilot override: follow the stick, re-latch target to where we are
                    self.depth_sp = self.depth
                    self.depth_pid.reset()
                else:
                    err = self.depth_sp - self.depth      # +err = need to go deeper
                    w.force.z = -self.depth_pid.update(err, self.dt)   # deeper = push down
            elif self.mode == HEADING_HOLD:
                if abs(p.angular.z) > db:
                    self.yaw_sp = self.yaw
                    self.heading_pid.reset()
                else:
                    err = wrap_pi(self.yaw_sp - self.yaw)
                    w.torque.z = self.heading_pid.update(err, self.dt)  # +err = turn CCW

        self.wrench_pub.publish(w)
        self.mode_pub.publish(String(data=self.mode))
        self.armed_pub.publish(Bool(data=self.mode != DISARMED))


def main():
    rclpy.init()
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
