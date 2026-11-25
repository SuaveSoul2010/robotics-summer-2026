import math
import numpy as np

import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")

        self.odom_sub = self.create_subscription(
            Odometry, "/ground_truth/state", self.odom_callback, 10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/occupancy_grid", self.map_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.goal_x = 5.0
        self.goal_y = 3.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.pose_received = False

        self.grid = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin_x = None
        self.map_origin_y = None

        self.avoiding_obstacle = False
        self.avoidance_direction = 1.0
        self.avoidance_target_angle = None

        self.get_logger().info("Navigator started")
        self.get_logger().info(f"Goal: ({self.goal_x}, {self.goal_y})")

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.robot_theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.pose_received = True

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height

        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            self.map_height, self.map_width
        )

    def world_to_grid(self, x, y):
        grid_x = int((x - self.map_origin_x) / self.map_resolution)

        grid_y = int((y - self.map_origin_y) / self.map_resolution)

        return grid_x, grid_y

    def obstacle_ahead(self):
        if self.grid is None:
            return False

        check_distances = np.arange(0.35, 1.2, 0.05)

        for distance in check_distances:
            check_x = self.robot_x + distance * math.cos(self.robot_theta)

            check_y = self.robot_y + distance * math.sin(self.robot_theta)

            grid_x, grid_y = self.world_to_grid(check_x, check_y)

            if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                continue

            if self.grid[grid_y, grid_x] == 100:
                return True

        return False

    def control_loop(self):
        if not self.pose_received:
            return

        if self.grid is None:
            return

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y

        distance_to_goal = math.sqrt(dx * dx + dy * dy)

        goal_angle = math.atan2(dy, dx)

        angle_error = self.normalize_angle(goal_angle - self.robot_theta)

        obstacle = self.obstacle_ahead()

        cmd = Twist()

        if distance_to_goal < 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.get_logger().info("Goal reached!")

        elif obstacle and not self.avoiding_obstacle:
            self.avoiding_obstacle = True

            # turn the opposite way from the goal side
            if angle_error >= 0.0:
                self.avoidance_direction = -1.0
            else:
                self.avoidance_direction = 1.0

            self.avoidance_target_angle = self.normalize_angle(
                self.robot_theta + self.avoidance_direction * math.radians(60.0)
            )

            self.get_logger().info(
                "Obstacle detected. " "Beginning deliberate avoidance turn."
            )

            cmd.linear.x = 0.0
            cmd.angular.z = self.avoidance_direction * 0.8

        elif self.avoiding_obstacle:
            turn_error = self.normalize_angle(
                self.avoidance_target_angle - self.robot_theta
            )

            if abs(turn_error) > math.radians(5.0):
                cmd.linear.x = 0.0
                cmd.angular.z = 1.0 * np.sign(turn_error)

            else:
                self.avoiding_obstacle = False
                self.avoidance_target_angle = None

                cmd.linear.x = 0.5
                cmd.angular.z = 0.0

                self.get_logger().info("Avoidance turn complete.")

        # too far off heading, turn in place first
        elif abs(angle_error) > math.radians(15.0):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 * angle_error

        else:
            cmd.linear.x = 0.7
            cmd.angular.z = 0.6 * angle_error

        self.cmd_vel_pub.publish(cmd)

        self.get_logger().info(
            f"Position: "
            f"({self.robot_x:.2f}, {self.robot_y:.2f}) | "
            f"Distance: "
            f"{distance_to_goal:.2f} m | "
            f"Angle error: "
            f"{math.degrees(angle_error):.1f}° | "
            f"Obstacle: "
            f"{obstacle} | "
            f"Avoiding: "
            f"{self.avoiding_obstacle}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = Navigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
