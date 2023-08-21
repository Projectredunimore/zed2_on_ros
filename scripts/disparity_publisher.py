#!/usr/bin/env python3


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge

class StereoDisparityNode:
    def __init__(self):
        rospy.init_node('stereo_disparity_node', anonymous=True)
        
        self.cv_bridge = CvBridge()
        
        self.left_image_sub = rospy.Subscriber('/left/image_rect', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('/right/image_rect', Image, self.right_image_callback)
        self.left_camera_info_sub = rospy.Subscriber('/left/camera_info', CameraInfo, self.left_camera_info_callback)
        self.right_camera_info_sub = rospy.Subscriber('/right/camera_info', CameraInfo, self.right_camera_info_callback)
        
        self.disparity_pub = rospy.Publisher('/disparity_image', DisparityImage, queue_size=10)
        
        self.left_camera_matrix = None
        self.right_camera_matrix = None
        self.baseline = None
        
    def left_image_callback(self, left_image_msg):
        try:
            left_image = self.cv_bridge.imgmsg_to_cv2(left_image_msg, "bgr8")
            self.process_disparity(left_image)
        except Exception as e:
            rospy.logerr("Error processing left image: %s", str(e))
        
    def right_image_callback(self, right_image_msg):
        try:
            right_image = self.cv_bridge.imgmsg_to_cv2(right_image_msg, "bgr8")
            self.process_disparity(right_image)
        except Exception as e:
            rospy.logerr("Error processing right image: %s", str(e))
            
    def left_camera_info_callback(self, camera_info_msg):
        self.left_camera_matrix = np.reshape(camera_info_msg.K, (3, 3))
        
    def right_camera_info_callback(self, camera_info_msg):
        self.right_camera_matrix = np.reshape(camera_info_msg.K, (3, 3))
        self.baseline = camera_info_msg.P[3] / camera_info_msg.P[0]  # Calculate baseline
        
    def process_disparity(self, image):
        if self.left_camera_matrix is None or self.right_camera_matrix is None or self.baseline is None:
            rospy.logwarn("Camera info not yet received. Skipping disparity computation.")
            return
        
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Stereo block matching parameters
        block_size = 15
        num_disparities = 64
        stereo = cv2.StereoSGBM_create(minDisparity=0,
                                       numDisparities=num_disparities,
                                       blockSize=block_size)
        
        # Compute the disparity map
        disparity = stereo.compute(gray_image, gray_image)
        
        # Normalize the disparity map for visualization
        normalized_disparity = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        
        # Create a DisparityImage message
        disparity_msg = DisparityImage()
        disparity_msg.header.stamp = rospy.Time.now()
        disparity_msg.image = self.cv_bridge.cv2_to_imgmsg(normalized_disparity, "mono8")
        disparity_msg.f = self.left_camera_matrix[0, 0]
        disparity_msg.min_disparity = 0
        disparity_msg.max_disparity = num_disparities
        disparity_msg.T = self.baseline
        
        # Publish the DisparityImage message
        self.disparity_pub.publish(disparity_msg)
        
if __name__ == '__main__':
    try:
        stereo_disparity_node = StereoDisparityNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass