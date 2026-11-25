import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")

        # Bridge between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Subscribe to camera feed
        self.image_subscriber = self.create_subscription(
            Image, "/front_camera/image_raw", self.image_callback, 10
        )

        # Publish processed black-and-white image
        self.image_publisher = self.create_publisher(
            Image, "/lane_detection/processed_image", 10
        )

        self.get_logger().info("Lane detector started")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # Convert to grayscale
        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Convert to black and white using thresholding
        _, binary_image = cv2.threshold(
            grayscale,
            150,  # Threshold value
            255,  # White pixel value
            cv2.THRESH_BINARY,
        )

        # Optional: Only keep the lower part of the image
        height, width = binary_image.shape
        binary_image[: height // 2, :] = 0

        # Display image using OpenCV
        cv2.imshow("Lane Detection", binary_image)
        cv2.waitKey(1)
        processed_msg = self.bridge.cv2_to_imgmsg(binary_image, encoding="mono8")
        processed_msg.header = msg.header

        # Publish processed image
        self.image_publisher.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
