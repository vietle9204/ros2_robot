#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import math
import numpy as np

FORWARD = 0
WALL_FOLLOW = 1

class FollowWallTest(Node):
    def __init__(self):
        super().__init__('follow_wall')
        self.sub = self.create_subscription(
            LaserScan,
            '/lidar',      # topic lidar
            self.lidar_callback,
            10
        )
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.state = FORWARD  # First state: moving forward
        self.get_logger().info('Follow Wall Node Initialized')
        self.twist_stamped = TwistStamped()

        self.safe_distance = 0.8  # m distance to start wall following
        self.linear_speed_max = 1.5 # m/s
        self.linear_step = 0.5 # m/s^2
        self.angular_speed_max = 0.5 # rad/s
        self.kt = 3.0  # coefficient for angular speed

#         self.vector_front_obstacle = (1.0, 0.0)  # tangent vector to front obstacle
#         self.vector_right_obstacle = (0.0, 0.0)  # tangent vector to right obstacle
# #        self.vector_left_obstacle = (0.0, 1.0)  # tangent vector to left obstacle
#         self.vector_robot = (1, 0)  # Vector Ox

    def lidar_callback(self, msg):
        ranges = msg.ranges
        mid_index = len(ranges) // 2
        right_index = len(ranges) // 4

#         angle_min = msg.angle_min
# #        angle_max = msg.angle_max
#         angle_increment = msg.angle_increment

    #     self.vector_front_obstacle = self.vector_2DPoint( 
    #        self.lidar_to_2DPoint(angle_min + (mid_index-10)*angle_increment ,ranges[mid_index-10]),
    #        self.lidar_to_2DPoint(angle_min + (mid_index+10)*angle_increment ,ranges[mid_index+10])
    #    )
    #     self.vector_right_obstacle = self.vector_2DPoint(
    #         self.lidar_to_2DPoint(angle_min + (right_index-25)*angle_increment ,ranges[right_index-25]),
    #         self.lidar_to_2DPoint(angle_min + (right_index+50)*angle_increment ,ranges[right_index+50])
    #     )

        # kiểm tra vật cản phía trước
        start = max(0, mid_index - 30)
        end = min(len(ranges), mid_index + 30)
        front_ranges = ranges[start:end] if end > start else [float('inf')]
        min_front = min(front_ranges)

        #ben
        start = max(0, right_index - 30)
        end = min(len(ranges), right_index + 30)
        right_ranges = ranges[start:end] if end > start else [float('inf')]
        min_right = min(right_ranges)

        if self.state == FORWARD:
            if min_front < self.safe_distance:
                self.get_logger().info('Obstacle detected! Switching to wall-follow')
                self.state = WALL_FOLLOW
                # stop briefly when switching
                self.twist_stamped.twist.linear.x = 0.0
                self.twist_stamped.twist.angular.z = 0.0
            else:
                self.twist_stamped.twist.linear.x = 1.0
                self.twist_stamped.twist.angular.z = 0.0

        if self.state == WALL_FOLLOW:
            # ví dụ bám tường bên phải 
            if min_front < self.safe_distance:
                self.get_logger().info('Obstacle detected!')
                self.twist_stamped.twist.linear.x = 0.0
                self.twist_stamped.twist.angular.z = 0.5
                self.get_logger().info('turning left: line=%.2f, ang=%.2f' % (self.twist_stamped.twist.linear.x, self.twist_stamped.twist.angular.z))
            else:   
                if min_right < 0.65:
                    self.twist_stamped.twist.angular.z = 0.5   # quay ra xa tường
                    self.twist_stamped.twist.linear.x = 0.3
                    self.get_logger().info('turning left: line=%.2f, ang=%.2f' % (self.twist_stamped.twist.linear.x, self.twist_stamped.twist.angular.z))
                elif min_right > 0.95:
                    self.twist_stamped.twist.angular.z = -0.5 # quay vào tường
                    self.twist_stamped.twist.linear.x = 0.3
                    self.get_logger().info('turning right: line=%.2f, ang=%.2f' % (self.twist_stamped.twist.linear.x, self.twist_stamped.twist.angular.z))
                else:
                #    if self.angle_between_vector(self.vector_robot, self.vector_right_obstacle) > 0.02:
                #        self.twist_stamped.twist.angular.z = 0.3  # chinh huong
                #        self.twist_stamped.twist.linear.x = 0.03
                #        self.get_logger().info('adjusting angle: line=%.2f, ang=%.2f' % (self.twist_stamped.twist.linear.x, self.twist_stamped.twist.angular.z))
                #    else:
                #        self.twist_stamped.twist.angular.z = 0.0  # bám tường
                #        self.twist_stamped.twist.linear.x = 0.8
                #        self.get_logger().info('going straight: line=%.2f, ang=%.2f' % (self.twist_stamped.twist.linear.x, self.twist_stamped.twist.angular.z))

                    dist_error = ranges[right_index+25]-ranges[right_index-25]
                    self.twist_stamped.twist.angular.z = max(min(-self.kt*(dist_error), self.angular_speed_max), -self.angular_speed_max)
                    self.twist_stamped.twist.linear.x = 0.05 if abs(dist_error) > 0.03 else 0.8
                    self.get_logger().info('follow_wall: line=%.2f, ang=%.2f' % (self.twist_stamped.twist.linear.x, self.twist_stamped.twist.angular.z))
        self.pub.publish(self.twist_stamped)

    # def lidar_to_2DPoint(self, angle, distance):
    #     x = distance*math.cos(angle)
    #     y = distance*math.sin(angle)
    #     return (x, y)

    # def vector_2DPoint(self, p1, p2):
    #     return (p2[0]-p1[0], p2[1]-p1[1])
    
    # def angle_between_vector(self, v1, v2):
    #     v1 = np.array(v1)
    #     v2 = np.array(v2)
    #     dot = np.dot(v1, v2)
    #     norm = np.linalg.norm(v1) * np.linalg.norm(v2)
    #     cos_theta = np.clip(dot / norm, -1.0, 1.0)
    #     theta = np.arccos(cos_theta)
    #     return theta  # radian


def main(args=None):
    rclpy.init(args=args)
    node = FollowWallTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
