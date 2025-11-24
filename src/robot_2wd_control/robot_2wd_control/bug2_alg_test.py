#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import math




def angle_normalize(z):
    return math.atan2(math.sin(z), math.cos(z))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Bug2Node(Node):
    def __init__(self):
        super().__init__('bug2_node')

        # Subscribers
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(LaserScan, '/lidar', self.lidar_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        # State
        self.odom = None            # (x, y, yaw)
        self.goal = None            # (x, y)
        self.start = None           # start point when goal was set
        self.lidar = None           # LaserScan message

        self.state = 'idle'         # 'idle', 'go_to_goal', 'wall_following'
        self.hit_point = None       # where we hit obstacle
        self.m_line_tol = 0.10      # tolerance to consider on m-line (meters)
        self.obstacle_dist = 0.8    # threshold to consider obstacle ahead
        self.influence_dist = 2.0
        self.goal_tol = 0.15        # goal reached tolerance
        self.max_lin = 1.0
        self.max_ang = 1.0

        # Control gains
        self.k_rho = 0.6
        self.k_alpha = 1.5
        self.wall_follow_k = 1.0
        self.desired_wall_dist = 0.45

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('bug2_node started')

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # convert quaternion to yaw
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (x, y, yaw)

    def goal_cb(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.goal = (gx, gy)
        if self.odom is not None:
            self.start = (self.odom[0], self.odom[1])
        self.hit_point = None
        self.state = 'go_to_goal'
        self.get_logger().info(f'New goal set: ({gx:.2f}, {gy:.2f}), starting go_to_goal')

    def lidar_cb(self, msg: LaserScan):
        self.lidar = msg

    def control_loop(self):
        if self.odom is None or self.goal is None:
            # nothing to do
            self.publish_stop()
            return

        robot_pos = (self.odom[0], self.odom[1], self.odom[2])
        goal_pos = self.goal

        # check goal reached
        if distance((robot_pos[0], robot_pos[1]), goal_pos) <= self.goal_tol:
            self.get_logger().info('Goal reached')
            self.state = 'idle'
            self.publish_stop()
            return

        if self.state == 'go_to_goal':
            twist = self.go_to_goal_control(robot_pos, goal_pos)
            # if obstacle ahead, switch to wall_following
            if self.is_obstacle_ahead():
                self.state = 'wall_following'
                self.hit_point = (robot_pos[0], robot_pos[1])
                self.get_logger().info('Obstacle ahead: switching to wall_following')
            self.cmd_pub.publish(twist)
            return

        if self.state == 'wall_following':
            # check leave condition: on m-line and closer to goal than hit_point
            if self.on_m_line(robot_pos) and self.hit_point is not None:
                current_dist_to_goal = distance((robot_pos[0], robot_pos[1]), goal_pos)
                hit_dist_to_goal = distance(self.hit_point, goal_pos)
                if math.fabs(current_dist_to_goal + 1e-3 - hit_dist_to_goal) < 0.1:
                    self.state = 'go_to_goal'
                    self.get_logger().info('On m-line and closer to goal than hit point: switching to go_to_goal')
                    return  # next loop will run go_to_goal

            twist = self.wall_follow_control(robot_pos)
            self.cmd_pub.publish(twist)
            return

        # idle
        self.publish_stop()

    def go_to_goal_control(self, robot_pos, goal_pos):
        x, y, yaw = robot_pos
        gx, gy = goal_pos
        dx = gx - x
        dy = gy - y
        rho = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        alpha = angle_normalize(angle_to_goal - yaw)

        v = self.k_rho * rho
        w = self.k_alpha * alpha

        # limit
        if v > self.max_lin:
            v = self.max_lin
        if w > self.max_ang:
            w = self.max_ang
        if w < -self.max_ang:
            w = -self.max_ang

        twist = TwistStamped()
        twist.twist.linear.x = v
        twist.twist.angular.z = w
        return twist

    def is_obstacle_ahead(self):
        if self.lidar is None:
            return False
        # check a frontal sector: +/- 30 degrees
        angles = []
        msg = self.lidar
        angle = msg.angle_min
        obstacle = False
        sector = math.radians(30)
        for r in msg.ranges:
            if math.isfinite(r):
                if -sector <= angle <= sector and r < self.obstacle_dist:
                    obstacle = True
                    break
            angle += msg.angle_increment
        return obstacle

    def wall_follow_control(self, robot_pos):
        twist = TwistStamped()
         # prepare sectors
        scan = self.lidar
        ranges = np.array(scan.ranges, dtype=np.float32)
        # thay thế inf/nan bằng range_max
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)

        # tạo mảng góc tương ứng với scan
        angles = np.linspace(scan.angle_min, scan.angle_max, ranges.size)  # rad
        
        # giới hạn góc từ -110 đến 110 độ
        angle_min = math.radians(-110)
        angle_max = math.radians(110)
        
        mask = (angles >= angle_min) & (angles <= angle_max) & (ranges < 2.0)
        # lọc
        filtered_ranges = ranges[mask]
        filtered_angles = angles[mask]

        # mask cho left: 0 -> +110°
        mask_left = filtered_angles >= 0.0
        obs_left_ranges = filtered_ranges[mask_left]
        obs_left_angles = filtered_angles[mask_left]

        # mask cho right: -110° -> 0°
        mask_right = filtered_angles < 0.0
        obs_right_ranges = filtered_ranges[mask_right]
        obs_right_angles = filtered_angles[mask_right]

        # Tính khoảng cách trung bình đến vật cản bên trái và bên phải
        left_dist = np.mean(obs_left_ranges) if obs_left_ranges.size > 0 else float('inf')
        right_dist = np.mean(obs_right_ranges) if obs_right_ranges.size > 0 else float('inf')   

        # quyết định bên theo dõi tường
        # if left_dist < right_dist:
        #     wall_side = 'left'
        #     wall_dist = left_dist
        #     # Left: đảo trước
        #     left_vectors = self.create_vectors(obs_left_ranges, obs_left_angles, reverse=True)
        #     left_arg_angle_diff = self.average_angle(left_vectors)
        #     z = -self.wall_follow_k*left_arg_angle_diff + -0.5*(min(0, self.desired_wall_dist - wall_dist))
        #     twist.twist.angular.z = min(self.max_ang, max(-self.max_ang, z))
        #     x = self.max_lin * (1 - abs(twist.twist.angular.z) / self.max_ang)
        #     twist.twist.linear.x = x
        # else:
        #     wall_side = 'right'
        #     wall_dist = right_dist
        #      # Right: không đảo
        #     right_vectors = self.create_vectors(obs_right_ranges, obs_right_angles, reverse=False)
        #     right_arg_angle_diff = self.average_angle(right_vectors)
        #     z = -self.wall_follow_k*right_arg_angle_diff + 0.5*(min(0, self.desired_wall_dist - wall_dist))
        #     twist.twist.angular.z = min(self.max_ang, max(-self.max_ang, z))
        #     x = self.max_lin * (1 - abs(twist.twist.angular.z) / self.max_ang)
        #     twist.twist.linear.x = x


        right_vectors = self.create_vectors(obs_right_ranges, obs_right_angles, reverse=False)
        right_arg_angle_diff = self.average_angle(right_vectors)
        z = -self.wall_follow_k*right_arg_angle_diff - 0.05*(min(0, self.desired_wall_dist - right_dist))
        twist.twist.angular.z = min(self.max_ang, max(-self.max_ang, z))
        x = self.max_lin * (1 - abs(twist.twist.angular.z) / self.max_ang)
        twist.twist.linear.x = x
        # # điều khiển theo dõi tường

        # error = self.desired_wall_dist - right_dist
        # angular_z = self.wall_follow_k * error


        return twist
    
    def create_vectors(self, obs_ranges, obs_angles, reverse=False):
        # """
        # Tạo mảng vector từ các điểm LiDAR.
        # obs_ranges, obs_angles: mảng đã lọc <2m
        # reverse: nếu True, đảo mảng trước khi tạo vector (dùng cho left)
        # """
        # điểm LiDAR sang (x, y)
        x = obs_ranges * np.cos(obs_angles)
        y = obs_ranges * np.sin(obs_angles)
        
        if reverse:
            x = x[::-1]
            y = y[::-1]
        
        vectors = []
        for i in range(len(x)-1):
            # nếu hai điểm khác nhau → tạo vector
            if x[i] != x[i+1] or y[i] != y[i+1]:
                vec = (x[i+1]-x[i], y[i+1]-y[i])
                vectors.append(vec)
        return np.array(vectors)  # mảng Nx2
    
    def average_angle(self, vectors):
        # """
        # Tính trung bình góc của các vector mà không dùng trọng số.
        # vectors: mảng Nx2 [(dx, dy), ...]
        # """
        if len(vectors) == 0:
            return 0.0
        angles = np.arctan2(vectors[:,1], vectors[:,0])
        return np.mean(angles)

    def on_m_line(self, robot_pos):
        # m-line: line from start to goal. Check perpendicular distance to that line.
        if self.start is None or self.goal is None:
            return False
        x0, y0 = self.start
        x1, y1 = self.goal
        x, y, _ = robot_pos

        # if start==goal (shouldn't happen), return False
        dx = x1 - x0
        dy = y1 - y0
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return False
        # distance from point to line (start->goal)
        dist = abs(dy * x - dx * y + x1 * y0 - y1 * x0) / norm
        return dist <= self.m_line_tol

    def publish_stop(self):
        t = TwistStamped()
        self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()