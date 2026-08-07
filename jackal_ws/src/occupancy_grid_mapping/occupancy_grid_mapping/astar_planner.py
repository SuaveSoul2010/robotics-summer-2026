import math
import heapq
import numpy as np
import matplotlib.pyplot as plt

import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ground_truth/state',
            self.odom_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',
            self.map_callback,
            10
        )

        self.has_planned = False
        
        self.goal_x = 5.0
        self.goal_y = 3.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.pose_received = False

        self.grid = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin_x = None
        self.map_origin_y = None

        self.path = None

        plt.ion()

        self.fig, self.ax = plt.subplots(
            figsize=(8, 8)
        )

        self.get_logger().info(
            'A* planner started'
        )

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        self.pose_received = True

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height

        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        self.grid = np.array(
            msg.data,
            dtype=np.int8
        ).reshape(
            self.map_height,
            self.map_width
        )

        if not self.has_planned:
            self.plan_path()
            self.has_planned = True

    def world_to_grid(self, x, y):
        grid_x = int(
            (x - self.map_origin_x)
            /
            self.map_resolution
        )

        grid_y = int(
            (y - self.map_origin_y)
            /
            self.map_resolution
        )

        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        x = (
            self.map_origin_x
            +
            (grid_x + 0.5)
            *
            self.map_resolution
        )

        y = (
            self.map_origin_y
            +
            (grid_y + 0.5)
            *
            self.map_resolution
        )

        return x, y

    def in_bounds(self, x, y):
        return (
            0 <= x < self.map_width
            and
            0 <= y < self.map_height
        )

    def is_walkable(self, x, y):
        if not self.in_bounds(x, y):
            return False

        return self.grid[y, x] != 100

    def heuristic(self, current, goal):
        dx = current[0] - goal[0]
        dy = current[1] - goal[1]

        return math.sqrt(
            dx * dx
            +
            dy * dy
        )

    def get_neighbors(self, current):
        x, y = current

        neighbors = [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
            (x + 1, y + 1),
            (x + 1, y - 1),
            (x - 1, y + 1),
            (x - 1, y - 1)
        ]

        return [
            neighbor
            for neighbor in neighbors
            if self.is_walkable(
                neighbor[0],
                neighbor[1]
            )
        ]

    def movement_cost(self, current, neighbor):
        dx = abs(
            current[0]
            -
            neighbor[0]
        )

        dy = abs(
            current[1]
            -
            neighbor[1]
        )

        if dx == 1 and dy == 1:
            return math.sqrt(2.0)

        return 1.0

    def a_star(self, start, goal):
        open_set = []

        heapq.heappush(
            open_set,
            (
                0.0,
                start
            )
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        f_score = {
            start: self.heuristic(
                start,
                goal
            )
        }

        while open_set:
            _, current = heapq.heappop(
                open_set
            )

            if current == goal:
                return self.reconstruct_path(
                    came_from,
                    current
                )

            for neighbor in self.get_neighbors(
                current
            ):
                tentative_g_score = (
                    g_score[current]
                    +
                    self.movement_cost(
                        current,
                        neighbor
                    )
                )

                if (
                    neighbor not in g_score
                    or
                    tentative_g_score
                    <
                    g_score[neighbor]
                ):
                    came_from[neighbor] = current

                    g_score[neighbor] = (
                        tentative_g_score
                    )

                    f_score[neighbor] = (
                        tentative_g_score
                        +
                        self.heuristic(
                            neighbor,
                            goal
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            f_score[neighbor],
                            neighbor
                        )
                    )

        return None

    def reconstruct_path(
        self,
        came_from,
        current
    ):
        path = [
            current
        ]

        while current in came_from:
            current = came_from[current]

            path.append(
                current
            )

        path.reverse()

        return path

    def plan_path(self):
        if not self.pose_received:
            return

        if self.grid is None:
            return

        start = self.world_to_grid(
            self.robot_x,
            self.robot_y
        )

        goal = self.world_to_grid(
            self.goal_x,
            self.goal_y
        )

        if not self.in_bounds(
            start[0],
            start[1]
        ):
            self.get_logger().warn(
                'Robot is outside map bounds'
            )

            return

        if not self.in_bounds(
            goal[0],
            goal[1]
        ):
            self.get_logger().warn(
                'Goal is outside map bounds'
            )

            return

        if not self.is_walkable(
            goal[0],
            goal[1]
        ):
            self.get_logger().warn(
                'Goal is occupied'
            )

            return

        self.get_logger().info(
            f'Planning from {start} to {goal}'
        )

        self.path = self.a_star(
            start,
            goal
        )

        if self.path is None:
            self.get_logger().warn(
                'No path found'
            )

        else:
            self.get_logger().info(
                f'Path found with '
                f'{len(self.path)} cells'
            )

        self.plot_map(
            start,
            goal
        )

    def plot_map(
        self,
        start,
        goal
    ):
        self.ax.clear()

        display_grid = np.full(
            (
                self.map_height,
                self.map_width
            ),
            0.5,
            dtype=float
        )

        display_grid[
            self.grid == 100
        ] = 0.0

        display_grid[
            self.grid == 0
        ] = 1.0

        self.ax.imshow(
            display_grid,
            cmap='gray',
            origin='lower',
            extent=[
                self.map_origin_x,
                self.map_origin_x
                +
                self.map_width
                *
                self.map_resolution,
                self.map_origin_y,
                self.map_origin_y
                +
                self.map_height
                *
                self.map_resolution
            ],
            vmin=0.0,
            vmax=1.0
        )

        if self.path is not None:
            path_x = []
            path_y = []

            for grid_x, grid_y in self.path:
                world_x, world_y = (
                    self.grid_to_world(
                        grid_x,
                        grid_y
                    )
                )

                path_x.append(
                    world_x
                )

                path_y.append(
                    world_y
                )

            self.ax.plot(
                path_x,
                path_y,
                linewidth=2
            )

        self.ax.plot(
            self.robot_x,
            self.robot_y,
            'ro',
            markersize=8
        )

        self.ax.plot(
            self.goal_x,
            self.goal_y,
            'gx',
            markersize=12,
            markeredgewidth=3
        )

        self.ax.set_title(
            'A* Path Planning'
        )

        self.ax.set_xlabel(
            'X (m)'
        )

        self.ax.set_ylabel(
            'Y (m)'
        )

        self.ax.set_aspect(
            'equal'
        )

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(
        args=args
    )

    node = AStarPlanner()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()