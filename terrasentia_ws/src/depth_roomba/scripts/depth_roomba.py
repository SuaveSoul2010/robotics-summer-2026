#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np

class DepthRoomba:
    def __init__(self):
        rospy.init_node("depth_roomba")
        self.bridge = CvBridge()

        # Publishers and subscribers
        self.cmd_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)

        self.sub = rospy.Subscriber(
            "/camera/depth/image_rect_raw", Image, self.depth_callback, queue_size=1
        )

        rospy.loginfo("Depth roomba started")

    def depth_callback(self, msg):
        rospy.loginfo("Received depth image")

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)
            return

        h, w = depth.shape

        # Look at the center of the image
        region = depth[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3]

        # Remove invalid depth values
        region = region[np.isfinite(region)]

        if len(region) == 0:
            rospy.logwarn("No valid depth values")
            return

        # RealSense depth is millimeters
        distance = np.mean(region) / 1000.0
        rospy.loginfo("Distance: %.2f m", distance)

        cmd = Twist()

        # Obstacle closer than 100 cm
        if distance < 1.0:
            rospy.loginfo("Obstacle detected - turning")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8

        else:
            cmd.linear.x = 0.25
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    try:
        DepthRoomba()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
