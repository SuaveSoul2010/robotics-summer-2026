#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


class OccupancyGridToGraph:
    def __init__(self):
        rospy.init_node("grid_to_graph_visualizer")

        # Parameters Downsample factor: 1 = every cell is a node (heavy in RViz) 2 = every 2nd cell, 4 = every 4th cell, etc.
        self.downsample_factor = 2

        # Grid threshold: cells with values < 0 or > free_threshold are ignored
        self.free_threshold = 0

        # Graph storage: { (gx, gy): [ ((neighbor_gx, neighbor_gy), cost), ... ] }
        self.graph = {}

        # ROS Subscribers & Publishers
        self.map_sub = rospy.Subscriber(
            "/occupancy_grid", OccupancyGrid, self.map_callback, queue_size=1
        )

        self.marker_pub = rospy.Publisher(
            "/graph_visualization", MarkerArray, queue_size=1, latch=True
        )

        rospy.loginfo("Grid-to-Graph Visualizer Node Initialized.")

    # Map Callback & Conversion Pipeline
    def map_callback(self, msg):
        rospy.loginfo("OccupancyGrid received. Converting to graph...")

        # Convert grid into adjacency graph data structure
        self.graph = self.convert_grid_to_graph(msg)

        rospy.loginfo("Graph construction complete! Total nodes: %d", len(self.graph))

        # Publish graph visualization to RViz
        self.publish_graph_markers(msg.info)

    # Grid -> Graph Data Structure Conversion
    def convert_grid_to_graph(self, grid_msg):
        info = grid_msg.info
        width = info.width
        height = info.height

        # Reshape flat map array into 2D grid (height x width)
        grid = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))

        # 8-neighbor movements: (delta_x, delta_y, cost)
        step = self.downsample_factor
        sqrt2 = 1.414 * step
        neighbors = [
            (0, step, step),
            (0, -step, step),
            (step, 0, step),
            (-step, 0, step),  # Cardinal
            (step, step, sqrt2),
            (-step, step, sqrt2),
            (step, -step, sqrt2),
            (-step, -step, sqrt2),  # Diagonal
        ]

        graph = {}

        # Iterate over downsampled grid coordinates
        for gy in range(0, height, step):
            for gx in range(0, width, step):

                # Only create nodes for known free cells
                if grid[gy, gx] != self.free_threshold:
                    continue

                node = (gx, gy)
                graph[node] = []

                # Connect to adjacent valid free cells
                for dx, dy, cost in neighbors:
                    nx, ny = gx + dx, gy + dy

                    # Boundary check
                    if 0 <= nx < width and 0 <= ny < height:
                        # Check if neighbor is free space
                        if grid[ny, nx] == self.free_threshold:
                            graph[node].append(((nx, ny), cost))

        return graph

    # RViz Marker Visualization
    def publish_graph_markers(self, info):
        marker_array = MarkerArray()

        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        # Marker 1: Graph Nodes (Points)
        nodes_marker = Marker()
        nodes_marker.header.stamp = rospy.Time.now()
        nodes_marker.header.frame_id = "odom"
        nodes_marker.ns = "graph_nodes"
        nodes_marker.id = 0
        nodes_marker.type = Marker.POINTS
        nodes_marker.action = Marker.ADD

        nodes_marker.scale.x = res * 0.4
        nodes_marker.scale.y = res * 0.4

        # Cyan / Blue color for nodes
        nodes_marker.color.a = 0.8
        nodes_marker.color.r = 0.0
        nodes_marker.color.g = 0.7
        nodes_marker.color.b = 1.0

        # Marker 2: Graph Edges (Lines)
        edges_marker = Marker()
        edges_marker.header.stamp = rospy.Time.now()
        edges_marker.header.frame_id = "odom"
        edges_marker.ns = "graph_edges"
        edges_marker.id = 1
        edges_marker.type = Marker.LINE_LIST
        edges_marker.action = Marker.ADD

        edges_marker.scale.x = res * 0.1  # Line thickness

        # Semi-transparent green for edges
        edges_marker.color.a = 0.3
        edges_marker.color.r = 0.0
        edges_marker.color.g = 1.0
        edges_marker.color.b = 0.0

        # Convert Grid Graph coordinates -> World Coordinates
        for (gx, gy), adj_list in self.graph.items():
            # World coordinate calculation
            wx = ox + (gx + 0.5) * res
            wy = oy + (gy + 0.5) * res

            p1 = Point(wx, wy, 0.05)
            nodes_marker.points.append(p1)

            for (nx, ny), _ in adj_list:
                # To prevent drawing duplicate bidirectional lines:
                if (gx, gy) < (nx, ny):
                    n_wx = ox + (nx + 0.5) * res
                    n_wy = oy + (ny + 0.5) * res
                    p2 = Point(n_wx, n_wy, 0.05)

                    edges_marker.points.append(p1)
                    edges_marker.points.append(p2)

        marker_array.markers.append(nodes_marker)
        marker_array.markers.append(edges_marker)

        self.marker_pub.publish(marker_array)


# Main Execution
if __name__ == "__main__":
    try:
        OccupancyGridToGraph()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
