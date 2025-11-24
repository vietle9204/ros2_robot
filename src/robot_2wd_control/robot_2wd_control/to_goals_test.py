#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

ODOM_TOPIC = '/diff_drive_controller/odom'   # subscribed odometry
GOAL_TOPIC = '/goal_pose'                   # subscribed goal (PoseStamped)
CMD_TOPIC = '/diff_drive_controller/cmd_vel'  # published cmd_vel (Twist)
             
class DiffDriveGoToGoal(Node):
    def __init__(self):
        super().__init__('to_goals_test')
        from sensor_msgs.msg import LaserScan  # local import so file top doesn't need changes

        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, GOAL_TOPIC, self.goal_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # goal state
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.have_goal = False

        # controller gains and limits
        self.kp_lin = 0.8
        self.kp_ang = 2.0
        self.max_lin = 0.6
        self.max_ang = 1.5
        self.tolerance = 0.03  # meters

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        # convert quaternion to yaw
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.goal_x = p.x
        self.goal_y = p.y
        self.have_goal = True
        self.get_logger().info(f"Received goal: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def shortest_angular_diff(self, target, source):
        a = target - source
        return (a + math.pi) % (2 * math.pi) - math.pi


    def control_loop(self):
        if not (self.have_goal):
            # publish zero to be safe
            self.cmd_pub.publish(TwistStamped())
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        # desired heading
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.shortest_angular_diff(target_yaw, self.yaw)

        cmd = TwistStamped()

        if dist < self.tolerance:
            # reached goal
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.have_goal = False
            self.get_logger().info("Goal reached.")
        else:
            # simple P controller: turn toward goal while moving
            cmd.twist.linear.x = max(0.0, min(self.max_lin, self.kp_lin * dist)) if abs(yaw_error) < 0.6 else 0.0
            cmd.twist.angular.z = max(-self.max_ang, min(self.max_ang, self.kp_ang * yaw_error))

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveGoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop robot before shutdown
        node.cmd_pub.publish(TwistStamped())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()