import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan


class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(LaserScan, '/front/scan', self.scan_cb, 10)

        self.latest_scan = None

        self.latest_imu = None
        self.latest_odom = None
        self.latest_joints = None

        self.timer = self.create_timer(0.5, self.print_data)

    def imu_cb(self, msg):
        self.latest_imu = msg

    def odom_cb(self, msg):
        self.latest_odom = msg

    def joint_cb(self, msg):
        self.latest_joints = msg

    def scan_cb(self, msg):
        self.latest_scan = msg

    def print_data(self):
        self.get_logger().info("---- SENSOR DATA ----")

        if self.latest_imu:
            self.get_logger().info(
                f"IMU z angular vel: {self.latest_imu.angular_velocity.z:.3f}"
            )

        if self.latest_odom:
            p = self.latest_odom.pose.pose.position
            self.get_logger().info(f"Odom: x={p.x:.2f}, y={p.y:.2f}")

        if self.latest_joints:
            self.get_logger().info(
                f"Joint count: {len(self.latest_joints.name)}"
            )
        
        if self.latest_scan:
            min_distance = min(self.latest_scan.ranges)
            self.get_logger().info(
                f"Closest obstacle distance: {min_distance:.2f} m"
            )

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

