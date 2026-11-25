#!/usr/bin/env python
# -*- coding: utf-8 -*-

import heapq
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path


class DijkstraPathPlanner:

    def __init__(self):
        rospy.init_node("dijkstra_planner")

        # Parameters
        self.obstacle_margin = 1  # Inflation margin (in grid cells)
        self.downsample_factor = 2  # Downsample step size (must match graph resolution)

        # State Variables
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.pose_received = False

        self.grid_msg = None
        self.grid_received = False

        # Subscribers & Publishers
        self.map_sub = rospy.Subscriber(
            "/occupancy_grid", OccupancyGrid, self.map_callback, queue_size=1
        )

        self.odom_sub = rospy.Subscriber(
            "/robot/odometry/local/raw", Odometry, self.odom_callback, queue_size=1
        )

        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback, queue_size=1
        )

        self.path_pub = rospy.Publisher("/calculated_path", Path, queue_size=1)

        rospy.loginfo(
            "Dijkstra Planner Node Initialized. Use '2D Nav Goal' in RViz to test."
        )

    # Callbacks
    def map_callback(self, msg):
        self.grid_msg = msg
        self.grid_received = True

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.pose_received = True

    def goal_callback(self, goal_msg):
        if not self.grid_received or not self.pose_received:
            rospy.logwarn("Planner: Missing grid map or robot pose. Cannot plan path.")
            return

        goal_x = goal_msg.pose.position.x
        goal_y = goal_msg.pose.position.y

        rospy.loginfo(
            "New goal received: (%.2f, %.2f). Running Dijkstra...", goal_x, goal_y
        )
        self.plan_and_publish_path(self.robot_x, self.robot_y, goal_x, goal_y)

    # Dijkstra Core Algorithm
    def run_dijkstra(self, start_node, goal_node, grid_array, width, height):
        step = self.downsample_factor
        sqrt2 = 1.414 * step

        # 8-connected movement vectors (dx, dy, step_cost)
        neighbors = [
            (0, step, step),
            (0, -step, step),
            (step, 0, step),
            (-step, 0, step),
            (step, step, sqrt2),
            (-step, step, sqrt2),
            (step, -step, sqrt2),
            (-step, -step, sqrt2),
        ]

        open_set = []
        heapq.heappush(open_set, (0.0, start_node))

        distances = {start_node: 0.0}
        parents = {}
        visited = set()

        while open_set:
            current_cost, curr = heapq.heappop(open_set)

            if curr in visited:
                continue
            visited.add(curr)

            if curr == goal_node:
                # Reconstruct path from goal to start
                path = []
                while curr in parents:
                    path.append(curr)
                    curr = parents[curr]
                path.append(start_node)
                path.reverse()
                return path

            cx, cy = curr
            for dx, dy, step_cost in neighbors:
                nx, ny = cx + dx, cy + dy

                if 0 <= nx < width and 0 <= ny < height:
                    # Cell value 0 indicates traversable free space
                    if grid_array[ny, nx] == 0:
                        neighbor = (nx, ny)
                        new_cost = current_cost + step_cost

                        if new_cost < distances.get(neighbor, float("inf")):
                            distances[neighbor] = new_cost
                            parents[neighbor] = curr
                            heapq.heappush(open_set, (new_cost, neighbor))

        return []

    # Path Resolution & Marker Publishing
    def plan_and_publish_path(self, rx, ry, gx, gy):
        info = self.grid_msg.info
        res = info.resolution
        w, h = info.width, info.height
        ox, oy = info.origin.position.x, info.origin.position.y

        # Convert raw OccupancyGrid data into 2D NumPy matrix
        raw_grid = np.array(self.grid_msg.data, dtype=np.int8).reshape((h, w))
        grid = np.copy(raw_grid)

        # Inflate obstacles to prevent close wall scraping
        if self.obstacle_margin > 0:
            occ_y, occ_x = np.where(raw_grid > 0)
            for y, x in zip(occ_y, occ_x):
                y_min, y_max = max(0, y - self.obstacle_margin), min(
                    h, y + self.obstacle_margin + 1
                )
                x_min, x_max = max(0, x - self.obstacle_margin), min(
                    w, x + self.obstacle_margin + 1
                )
                grid[y_min:y_max, x_min:x_max] = 100

        # Map world coordinates to grid indices snapped to downsample step
        step = self.downsample_factor
        sgx = int((rx - ox) / res) // step * step
        sgy = int((ry - oy) / res) // step * step
        ggx = int((gx - ox) / res) // step * step
        ggy = int((gy - oy) / res) // step * step

        # Ensure start and goal indices stay within grid bounds
        sgx, sgy = np.clip(sgx, 0, w - 1), np.clip(sgy, 0, h - 1)
        ggx, ggy = np.clip(ggx, 0, w - 1), np.clip(ggy, 0, h - 1)

        # Clear a step-sized window around start and goal to avoid trapping
        r = step
        grid[
            max(0, sgy - r) : min(h, sgy + r + 1), max(0, sgx - r) : min(w, sgx + r + 1)
        ] = 0
        grid[
            max(0, ggy - r) : min(h, ggy + r + 1), max(0, ggx - r) : min(w, ggx + r + 1)
        ] = 0

        # Execute Graph Search
        grid_path = self.run_dijkstra((sgx, sgy), (ggx, ggy), grid, w, h)

        if not grid_path:
            rospy.logwarn("Dijkstra failed: No valid path exists to target cell.")
            return

        # Construct ROS Path Message
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"

        for cell_x, cell_y in grid_path:
            wx = ox + (cell_x + 0.5) * res
            wy = oy + (cell_y + 0.5) * res

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Path published successfully with %d waypoints.", len(grid_path))


if __name__ == "__main__":
    try:
        planner = DijkstraPathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
