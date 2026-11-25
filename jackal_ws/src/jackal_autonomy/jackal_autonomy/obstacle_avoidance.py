import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        self.scan_sub = self.create_subscription(
            LaserScan, "/front/scan", self.scan_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Obstacle avoidance started")

    def scan_callback(self, msg):
        # Remove invalid readings
        ranges = [r for r in msg.ranges if r > 0]

        if not ranges:
            return

        closest = min(ranges)
        cmd = Twist()

        if closest < 1.0:
            # obstacle nearby: turn
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0
            self.get_logger().info(f"Obstacle {closest:.2f}m away - turning")

        else:
            # clear path: move forward
            cmd.linear.x = 1.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
