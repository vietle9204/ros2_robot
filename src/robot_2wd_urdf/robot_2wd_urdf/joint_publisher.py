#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')

        # Publisher /joint_states
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        self.start_time = time.time()
        self.get_logger().info('Joint Publisher started')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # chỉ 2 joint cần điều khiển
        msg.name = ['left_wheel_joint', 'right_wheel_joint']

        t = time.time() - self.start_time
        angle = math.sin(t * 2.0)  # dao động theo thời gian

        msg.position = [angle, -angle]  # quay ngược chiều nhau
        msg.velocity = []
        msg.effort = []

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

