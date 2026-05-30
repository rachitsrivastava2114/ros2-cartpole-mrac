"""
Cart-Pole MRAC CSV Error Logger
===============================
ROS 2 node that subscribes to /joint_states and logs only:

  - timestamp
  - cart_position_error = cart_position_ref (0) - cart_position
  - pole_angle_error    = pole_angle_ref (0) - pole_angle

CSV is written to:
  ~/cart_pole_mrac_error_<YYYYMMDD_HHMMSS>.csv
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CartPoleMRACCsvLogger(Node):

    def __init__(self):
        super().__init__('cart_pole_mrac_csv_logger')

        self._cart_idx = None
        self._pole_idx = None

        self.cart_position_ref = 0.0
        self.pole_angle_ref = 0.0

        self._rows = []
        self._start_time = self.get_clock().now()

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(
            os.path.expanduser('~'),
            f'cart_pole_mrac_error_{timestamp}.csv'
        )

        self._sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        self.get_logger().info(
            f'MRAC CSV error logger started. Data will be saved to: {self._csv_path}'
        )

    def _joint_state_cb(self, msg: JointState):
        if self._cart_idx is None or self._pole_idx is None:
            try:
                self._cart_idx = list(msg.name).index('cart_joint')
                self._pole_idx = list(msg.name).index('pole_joint')
            except ValueError:
                return

        ci, pi = self._cart_idx, self._pole_idx

        try:
            t = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

            cart_position = msg.position[ci]

            pole_angle = math.atan2(
                math.sin(msg.position[pi]),
                math.cos(msg.position[pi])
            )

            cart_position_error = self.cart_position_ref - cart_position
            pole_angle_error = self.pole_angle_ref - pole_angle

            self._rows.append({
                'time_s': round(t, 6),
                'cart_position_error': round(cart_position_error, 6),
                'pole_angle_error': round(pole_angle_error, 6),
            })

        except IndexError:
            pass

    def _write_csv(self):
        if not self._rows:
            self.get_logger().warn('No data collected. CSV not written.')
            return

        fieldnames = [
            'time_s',
            'cart_position_error',
            'pole_angle_error',
        ]

        with open(self._csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._rows)

        self.get_logger().info(
            f'MRAC error CSV saved: {self._csv_path} ({len(self._rows)} rows)'
        )

    def destroy_node(self):
        self._write_csv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CartPoleMRACCsvLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()