#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthColorizer:
    def __init__(self):
        rospy.init_node('depth_colorizer')
        self.bridge = CvBridge()
        
        # Subscribe to the raw 16-bit depth image
        self.sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.callback)
        
        # Publish the colorized version for Darknet
        self.pub = rospy.Publisher('/camera/depth_colorized', Image, queue_size=1)

    def callback(self, msg):
        # 1. Convert ROS Image to OpenCV format (16-bit)
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # 2. Scale and Normalize: Darknet needs 8-bit (0-255)
        # We clip it to 5 meters to make nearby boxes very distinct
        max_dist = 5000.0 # 5 meters in mm
        depth_clipped = np.clip(depth_image, 0, max_dist)
        depth_scaled = (depth_clipped / max_dist * 255).astype(np.uint8)

        # 3. Apply a colormap (JET makes things "pop" for AI detection)
        color_depth = cv2.applyColorMap(depth_scaled, cv2.COLORMAP_JET)

        # 4. Convert back to ROS and publish
        out_msg = self.bridge.cv2_to_imgmsg(color_depth, encoding="bgr8")
        out_msg.header = msg.header # Keep the time stamp for sync!
        self.pub.publish(out_msg)

if __name__ == '__main__':
    node = DepthColorizer()
    rospy.spin()