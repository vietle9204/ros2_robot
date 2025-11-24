#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class JointPublisher2WD(Node):
    def __init__(self):
        super().__init__('joint_publisher_2wd')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        self.left_pos = 0.0
        self.right_pos = 0.0
        
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().to_msg()

        # Tốc độ góc (rad/s)
        self.left_wheel_vel = 1.0
        self.right_wheel_vel = 1.0
        self.get_logger().info('Joint Publisher 2WD started.')

    def timer_callback(self):
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9  # tổng số giây
        
        # Tính góc quay theo thời gian
        self.left_pos = self.left_wheel_vel * t
        self.right_pos = self.right_wheel_vel * t
        
        msg = JointState()
        msg.header.stamp = now.to_msg() 
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.left_pos, self.right_pos]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher2WD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

