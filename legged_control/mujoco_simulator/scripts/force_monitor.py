#!/usr/bin/env python3
"""Monitor MPC contact forces (Z-axis) for all 4 feet."""
import rclpy
from rclpy.node import Node
from legged_msgs.msg import MpcObservation

class ForceMonitor(Node):
    def __init__(self):
        super().__init__('force_monitor')
        self.sub = self.create_subscription(
            MpcObservation, 'legged_robot_mpc_observation', self.cb, 10)
        self.count = 0

    def cb(self, msg):
        val = msg.input.value
        lf_z = val[2]
        rf_z = val[5]
        lh_z = val[8]
        rh_z = val[11]
        self.count += 1
        if self.count % 20 == 0:  # print every 20 msgs (~2Hz given 40Hz MPC)
            self.get_logger().info(
                f'Z forces: LF={lf_z:+.1f}  RF={rf_z:+.1f}  LH={lh_z:+.1f}  RH={rh_z:+.1f} N')

def main():
    rclpy.init()
    node = ForceMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
