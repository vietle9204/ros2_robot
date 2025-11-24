#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

class BugAlgNode(Node):
    def __init__(self):
        super().__init__('bug_alg_test')
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        self.create_subscription(LaserScan, '/lidar', self.lidar_cb, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.get_logger().info('Bug Algorithm Node Initialized')

        # parameters (tweak as needed)
        self.forward_speed_max = 1.5
        self.turn_speed_max = 0.6
        self.linear_step = 0.5 # m/s^2
        self.kt = 3.0  # coefficient for angular speed

        # robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # goal pose (updated via /goal_pose)
        self.has_goal = False
        self.goal_x = 0.0
        self.goal_y = 0.0

        #obstacle avoidance
        self.safe_distance = 0.8  # m distance to start wall following
        # self.vector_front_obstacle = (1.0, 0.0)  # tangent vector to front obstacle
        # self.vector_right_obstacle = (0.0, -1.0)  # tangent vector to right obstacle
        # self.vector_left_obstacle = (0.0, 1.0)  # tangent vector to left obstacle
        # self.vector_base_lidar = (1, 0)

        # robot
        self.state = 'NONE'  # Initial state


    def lidar_cb(self, msg: LaserScan):
        if(self.has_goal):    
            if(self.state == 'NONE'):
                self.state = 'GO_TO_GOAL'

            ranges = msg.ranges
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            mid_index = len(ranges) // 2
            right_index = len(ranges) // 4
            left_index = 3 * len(ranges) // 4

            # self.vector_front_obstacle = self.vector_2DPoint( 
            #     self.lidar_to_2DPoint(angle_min + (mid_index-10)*angle_increment ,ranges[mid_index-10]),
            #     self.lidar_to_2DPoint(angle_min + (mid_index+10)*angle_increment ,ranges[mid_index+10])
            # )
            # self.vector_right_obstacle = self.vector_2DPoint(
            #     self.lidar_to_2DPoint(angle_min + (right_index-25)*angle_increment ,ranges[right_index-25]),
            #     self.lidar_to_2DPoint(angle_min + (right_index+25)*angle_increment ,ranges[right_index+25])
            # )
            # self.vector_left_obstacle = self.vector_2DPoint(
            #     self.lidar_to_2DPoint(angle_min + (left_index-25)*angle_increment ,ranges[left_index-25]),
            #     self.lidar_to_2DPoint(angle_min + (left_index+25)*angle_increment ,ranges[left_index+25])
            # )

            #phai
            start = max(0, right_index - 30)
            end = min(len(ranges), right_index + 70)
            right_ranges = ranges[start:end] if end > start else [float('inf')]
            min_right = min(right_ranges)
            if(min_right < self.safe_distance):
                self.get_logger().info('Obstacle right!')
                self.state = 'OBSTACLE_AVOIDANCE_RIGHT'

            #trai
            start = max(0, left_index - 70)
            end = min(len(ranges), left_index + 30)
            left_ranges = ranges[start:end] if end > start else [float('inf')]
            min_left = min(left_ranges)
            if(min_left < self.safe_distance):
                self.get_logger().info('Obstacle left!')
                self.state = 'OBSTACLE_AVOIDANCE_LEFT'

            # kiểm tra vật cản phía trước
            start = max(0, mid_index - 70)
            end = min(len(ranges), mid_index + 70)
            front_ranges = ranges[start:end] if end > start else [float('inf')]
            min_front = min(front_ranges)
            if(min_front < self.safe_distance):
                self.get_logger().info('Obstacle ahead!')
                self.state = 'OBSTACLE_AVOIDANCE'

            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()

            match self.state:  
                case 'OBSTACLE_AVOIDANCE_RIGHT': 
                    twist.twist.linear.x = 0.2
                    twist.twist.angular.z = 0.3
                    self.state = 'GO_TO_GOAL'
                    self.get_logger().info('Avoiding obstacle on the right: turning left.')

                case 'OBSTACLE_AVOIDANCE_LEFT':
                    twist.twist.linear.x = 0.2
                    twist.twist.angular.z = -0.3
                    self.state = 'GO_TO_GOAL'
                    self.get_logger().info('Avoiding obstacle on the left: turning right.')

                case 'OBSTACLE_AVOIDANCE':
                    twist.twist.linear.x = 0.0
                    twist.twist.angular.z = 0.5
                    self.state = 'FOLLOW_WALL'
                    self.get_logger().info('Avoiding obstacle ahead: turning left.')

                case 'GO_TO_GOAL':
                    dx = self.goal_x - self.x
                    dy = self.goal_y - self.y
                    dist = math.hypot(dx, dy)

                    # desired heading
                    target_yaw = math.atan2(dy, dx)
                    yaw_error = self.shortest_angular_diff(target_yaw, self.yaw) 

                    if dist < 0.1:
                        # reached goal
                        twist.twist.linear.x = 0.0
                        twist.twist.angular.z = 0.0
                        self.has_goal = False
                        self.state = 'NONE'
                        self.get_logger().info("[To goal] Goal reached.")
                    else:
                        lidar_goal_index = int(len(ranges)/2 + yaw_error/angle_increment)
                        if ranges[lidar_goal_index] < 1.5*self.safe_distance:
                            self.get_logger().info('[To goal] Path to goal blocked by obstacle!')
                            self.state = 'FOLLOW_WALL'
                            twist.twist.linear.x = 0.0
                            twist.twist.angular.z = 0.0
                        else:
                            twist.twist.linear.x = max(0.0, min(self.forward_speed_max, 0.5 * dist)) if abs(yaw_error) < 0.5 else 0.0
                            twist.twist.angular.z = max(-self.turn_speed_max, min(self.turn_speed_max, 2.0 * yaw_error))
                            self.get_logger().info('[To goal] Moving to goal: dist=%.2f, yaw_err=%.2f' % (dist, yaw_error))
                
                case 'FOLLOW_WALL':
                    dx = self.goal_x - self.x
                    dy = self.goal_y - self.y
                    dist = math.hypot(dx, dy)

                    # desired heading
                    target_yaw = math.atan2(dy, dx)
                    yaw_error = self.shortest_angular_diff(target_yaw, self.yaw)

                    lidar_goal_index = int(len(ranges)/2 + yaw_error/angle_increment)
                    if ranges[lidar_goal_index] > 2*self.safe_distance:
                        self.get_logger().info('[Following wall] Path to goal clear!')
                        self.state = 'GO_TO_GOAL'
#                    elif min_front < self.safe_distance:
#                        twist.twist.linear.x = 0.0
#                        twist.twist.angular.z = 0.5
                    elif min_right > 1.3 * self.safe_distance:
                        twist.twist.angular.z = -0.3 # quay vao tuong
                        twist.twist.linear.x = 0.3
                        self.get_logger().info('[Following wall] too far wall. line=%.2f, ang=%.2f' % (twist.twist.linear.x, twist.twist.angular.z))
                    else:
                        # angle_between_robot_obs = self.angle_between_vectors(self.vector_base_lidar, self.vector_right_obstacle)    
                        # if (math.fabs(angle_between_robot_obs) > 0.05):
                        #     twist.twist.angular.z = min(self.turn_speed_max, 0.5*angle_between_robot_obs)
                        #     twist.twist.linear.x = 0.05
                        #     self.get_logger().info('[Following wall] adjusting angle. line=%.2f, ang=%.2f' % (twist.twist.linear.x, twist.twist.angular.z))
                        # else:
                        #     twist.twist.linear.x = 0.8
                        #     twist.twist.angular.z = 0.0
                        #     self.get_logger().info('[Following wall] moving forward. line=%.2f, ang=%.2f' % (twist.twist.linear.x, twist.twist.angular.z))

                        dist_error = ranges[right_index+25]-ranges[right_index-25]
                        twist.twist.angular.z = max(min(-self.kt*(dist_error), self.turn_speed_max), -self.turn_speed_max)
                        twist.twist.linear.x = 0.05 if abs(dist_error) > 0.03 else 0.8
                        self.get_logger().info('[follow_wall]: line=%.2f, ang=%.2f' % (twist.twist.linear.x, twist.twist.angular.z))
                case _:
                    twist.twist.linear.x = 0.0
                    twist.twist.angular.z = 0.0                 

            self.cmd_pub.publish(twist)

    def shortest_angular_diff(self, target, source):
        a = target - source
        return (a + math.pi) % (2 * math.pi) - math.pi

    # def lidar_to_2DPoint(self, angle, distance):
    #     x = distance*math.cos(angle)
    #     y = distance*math.sin(angle)
    #     return (x, y)
    
    # def vector_2DPoint(self, p1, p2):
    #     return (p2[0]-p1[0], p2[1]-p1[1])
    
    # def angle_between_vectors(self, v1, v2):
    #     cross = v1[0]*v2[1] - v1[1]*v2[0]
    #     dot = v1[0]*v2[0] + v1[1]*v2[1]
    #     return math.atan2(cross, dot)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        # convert quaternion to yaw
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_cb(self, msg):
        # msg is a geometry_msgs/PoseStamped
        p = msg.pose.position
        self.goal_x = p.x
        self.goal_y = p.y
        self.state = 'GO_TO_GOAL'
        self.get_logger().info(f"Received goal: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        self.has_goal = True


def main(args=None):
    rclpy.init(args=args)
    node = BugAlgNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()