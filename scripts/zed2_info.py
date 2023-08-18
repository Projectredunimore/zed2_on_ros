#!/usr/bin/env python3
import rospy
import yaml
from sensor_msgs.msg import CameraInfo

class Info():
    def __init__(self):
        rospy.init_node('zed2_info', anonymous=True)
        left_file = rospy.get_param('~left_file')
        right_file = rospy.get_param('~right_file')
        left_frame_id = rospy.get_param('~left_frame_id')
        right_frame_id = rospy.get_param('~right_frame_id')
        self.left_pub = rospy.Publisher('/zed2/left/camera_info', CameraInfo, queue_size=10)
        self.right_pub = rospy.Publisher('/zed2/right/camera_info', CameraInfo, queue_size=10)
        self.left_info_msg = CameraInfo()
        self.right_info_msg = CameraInfo()
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        print("left file: ", left_file)
        print("right file: ", right_file)
        print("left frame id: ", left_frame_id)
        print("right frame id: ", right_frame_id)

        # Read calibration data from yaml file
        with open(left_file, 'r') as f:
            self.data_left = yaml.load(f)

        with open(right_file, 'r') as f:
            self.data_right = yaml.load(f)

        # Create left CameraInfo message
        self.left_info_msg.header.frame_id = left_frame_id
        self.left_info_msg.height = self.data_left['image_height']
        self.left_info_msg.width = self.data_left['image_width']
        self.left_info_msg.distortion_model = self.data_left['distortion_model']
        self.left_info_msg.D = self.data_left['distortion_coefficients']['data']
        self.left_info_msg.K = self.data_left['camera_matrix']['data']
        self.left_info_msg.R = self.data_left['rectification_matrix']['data']
        self.left_info_msg.P = self.data_left['projection_matrix']['data']

        # Create right CameraInfo message
        self.right_info_msg.header.frame_id = right_frame_id
        self.right_info_msg.height = self.data_right['image_height']
        self.right_info_msg.width = self.data_right['image_width']
        self.right_info_msg.distortion_model = self.data_right['distortion_model']
        self.right_info_msg.D = self.data_right['distortion_coefficients']['data']
        self.right_info_msg.K = self.data_right['camera_matrix']['data']
        self.right_info_msg.R = self.data_right['rectification_matrix']['data']
        self.right_info_msg.P = self.data_right['projection_matrix']['data']

    def timer_callback(self, event):
        self.left_info_msg.header.stamp = rospy.Time.now()
        self.right_info_msg.header.stamp = rospy.Time.now()
        self.left_pub.publish(self.left_info_msg)
        self.right_pub.publish(self.right_info_msg)
        


def main():
    info = Info()
    rospy.spin()


if __name__ == "__main__":
    main()