import rclpy
import math
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped

LOG_ODDS_FREE = -0.4
LOG_ODDS_OCCUPIED = 0.85
LOG_ODDS_MIN = -5.0
LOG_ODDS_MAX = 5.0
LOG_ODDS_THRESHOLD = 0.0


class OccupancyMapper(Node):
    def __init__(self):
        super().__init__("occupancy_mapper")

        self.scan_sub = self.create_subscription(
            LaserScan, "/front/scan", self.scan_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/ground_truth/state", self.odom_callback, 10
        )

        self.map_pub = self.create_publisher(OccupancyGrid, "/occupancy_grid", 10)

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.pose_received = False

        self.goal_x = None
        self.goal_y = None

        self.map_size_x = 20.0
        self.map_size_y = 20.0
        self.resolution = 0.05
        self.max_mapping_range = 8.0

        self.grid_width = int(self.map_size_x / self.resolution)
        self.grid_height = int(self.map_size_y / self.resolution)

        self.log_odds = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)

        self.trajectory_x = []
        self.trajectory_y = []

        plt.ion()

        self.fig, self.ax = plt.subplots(figsize=(8, 8))

        self.fig.canvas.mpl_connect("button_press_event", self.map_clicked)

        self.get_logger().info("Occupancy mapper started")

        self.get_logger().info(f"Grid size: {self.grid_width} x {self.grid_height}")

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.robot_theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.pose_received = True

    def world_to_grid(self, x, y):
        grid_x = int((x + self.map_size_x / 2.0) / self.resolution)

        grid_y = int((y + self.map_size_y / 2.0) / self.resolution)

        return grid_x, grid_y

    def in_bounds(self, grid_x, grid_y):
        return 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height

    def bresenham_line(self, x0, y0, x1, y1):
        points = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        x = x0
        y = y0

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        if dx > dy:
            err = dx / 2.0

            while x != x1:
                points.append((x, y))

                err -= dy

                if err < 0:
                    y += sy
                    err += dx

                x += sx

        else:
            err = dy / 2.0

            while y != y1:
                points.append((x, y))

                err -= dx

                if err < 0:
                    x += sx
                    err += dy

                y += sy

        points.append((x1, y1))

        return points

    def update_free_cell(self, grid_x, grid_y):
        if not self.in_bounds(grid_x, grid_y):
            return

        self.log_odds[grid_y, grid_x] = max(
            LOG_ODDS_MIN, self.log_odds[grid_y, grid_x] + LOG_ODDS_FREE
        )

    def update_occupied_cell(self, grid_x, grid_y):
        if not self.in_bounds(grid_x, grid_y):
            return

        self.log_odds[grid_y, grid_x] = min(
            LOG_ODDS_MAX, self.log_odds[grid_y, grid_x] + LOG_ODDS_OCCUPIED
        )

    def map_clicked(self, event):
        if event.inaxes != self.ax:
            return

        if event.xdata is None or event.ydata is None:
            return

        self.goal_x = event.xdata
        self.goal_y = event.ydata

        goal_msg = PoseStamped()

        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "odom"

        goal_msg.pose.position.x = self.goal_x
        goal_msg.pose.position.y = self.goal_y
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)

        self.get_logger().info(
            f"New goal selected: " f"({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def scan_callback(self, msg):
        if not self.pose_received:
            return

        robot_x = self.robot_x
        robot_y = self.robot_y
        robot_theta = self.robot_theta

        # lidar is about 12 cm ahead of the robot's center
        laser_x = robot_x + 0.12 * math.cos(robot_theta)

        laser_y = robot_y + 0.12 * math.sin(robot_theta)

        laser_grid_x, laser_grid_y = self.world_to_grid(laser_x, laser_y)

        if not self.in_bounds(laser_grid_x, laser_grid_y):
            return

        robot_grid_x, robot_grid_y = self.world_to_grid(robot_x, robot_y)

        self.update_free_cell(robot_grid_x, robot_grid_y)

        valid_measurements = 0
        occupied_measurements = 0

        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min:
                continue

            is_hit = distance <= self.max_mapping_range and distance < msg.range_max

            ray_distance = distance if is_hit else self.max_mapping_range

            valid_measurements += 1

            laser_angle = msg.angle_min + i * msg.angle_increment

            world_angle = robot_theta + laser_angle

            endpoint_x = laser_x + ray_distance * math.cos(world_angle)

            endpoint_y = laser_y + ray_distance * math.sin(world_angle)

            endpoint_grid_x, endpoint_grid_y = self.world_to_grid(
                endpoint_x, endpoint_y
            )

            endpoint_grid_x = max(0, min(self.grid_width - 1, endpoint_grid_x))

            endpoint_grid_y = max(0, min(self.grid_height - 1, endpoint_grid_y))

            ray = self.bresenham_line(
                laser_grid_x, laser_grid_y, endpoint_grid_x, endpoint_grid_y
            )

            for free_x, free_y in ray[:-1]:
                self.update_free_cell(free_x, free_y)

            if is_hit:
                self.update_occupied_cell(endpoint_grid_x, endpoint_grid_y)

                occupied_measurements += 1

        self.trajectory_x.append(robot_x)
        self.trajectory_y.append(robot_y)

        self.publish_map()
        self.plot_map(robot_x, robot_y, robot_theta)

    def publish_map(self):
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        msg.info.resolution = self.resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height

        msg.info.origin.position.x = -self.map_size_x / 2.0

        msg.info.origin.position.y = -self.map_size_y / 2.0

        msg.info.origin.orientation.w = 1.0

        grid_out = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)

        grid_out[self.log_odds > LOG_ODDS_THRESHOLD] = 100

        grid_out[self.log_odds < 0.0] = 0  # free, the rest stays -1 (unknown)

        msg.data = grid_out.flatten().tolist()

        self.map_pub.publish(msg)

    def plot_map(self, robot_x, robot_y, robot_theta):
        self.ax.clear()

        display_grid = np.full((self.grid_height, self.grid_width), 0.5, dtype=float)

        display_grid[self.log_odds > LOG_ODDS_THRESHOLD] = 0.0

        display_grid[self.log_odds < 0.0] = 1.0

        self.ax.imshow(
            display_grid,
            cmap="gray",
            origin="lower",
            extent=[
                -self.map_size_x / 2.0,
                self.map_size_x / 2.0,
                -self.map_size_y / 2.0,
                self.map_size_y / 2.0,
            ],
            vmin=0.0,
            vmax=1.0,
        )

        if len(self.trajectory_x) > 1:
            self.ax.plot(self.trajectory_x, self.trajectory_y, linewidth=1)

        self.ax.plot(robot_x, robot_y, "ro", markersize=7)

        if self.goal_x is not None:
            self.ax.plot(
                self.goal_x, self.goal_y, "gx", markersize=12, markeredgewidth=3
            )

        arrow_length = 0.5

        self.ax.arrow(
            robot_x,
            robot_y,
            arrow_length * math.cos(robot_theta),
            arrow_length * math.sin(robot_theta),
            head_width=0.15,
            length_includes_head=True,
        )

        self.ax.set_title("Occupancy Grid Map - Click to Select Goal")

        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)

    node = OccupancyMapper()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
