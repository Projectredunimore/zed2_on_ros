#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import Image
import rospkg

from cv_bridge import CvBridge, CvBridgeError
import cv2
import datetime



class CLI():
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)

        # get cameras names from launch params
        self.cam1 = rospy.get_param('~cam1_name', 'zed2')
        self.cam2 = rospy.get_param('~cam2_name', 'd435_left')
        self.cam3 = rospy.get_param('~cam3_name', 'd435_right')

        # check services exist
        self.sub1 = rospy.Subscriber("camera1/color/image_raw", Image, self.image_callback_1)
        self.sub2 = rospy.Subscriber("camera2/color/image_raw", Image, self.image_callback_2)
        self.sub3 = rospy.Subscriber("camera3/color/image_raw", Image, self.image_callback_3)

        self.cv_image1 = None
        self.cv_image2 = None
        self.cv_image3 = None

        self.bridge = CvBridge()
        self.path = rospkg.RosPack().get_path('zed2_on_ros')

    def image_callback_1(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    def image_callback_2(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        
    def image_callback_3(self, data):
        try:
            self.cv_image3 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    
    def interface(self):
        print('Welcome to image saver CLI')
        print('Type the number of the camera you want to save the image from')
        print('1. {}'.format(self.cam1))
        print('2. {}'.format(self.cam2))
        print('3. {}'.format(self.cam3))
        print('0. Exit')

        while not rospy.is_shutdown():
            img = None
            try:
                cam = int(input('Camera: '))
                if cam == 1:
                    img = self.cv_image1
                elif cam == 2:
                    img = self.cv_image2
                elif cam == 3:
                    img = self.cv_image3
                elif cam == 0:
                    break
                else:
                    print('Invalid option')
            except ValueError:
                print('Invalid option')

            else:
                time = datetime.datetime.now()
                y = time.year
                m = time.month
                d = time.day
                h = time.hour
                minute = time.minute
                s = time.second

                img_name = '{}-{}-{}-{}-{}-{}_cam{}.png'.format(y, m, d, h, minute, s, cam)
                img_path = self.path + '/images/' + img_name

                print('Saving image {} from camera {}'.format(img_name, cam))
                cv2.imwrite(img_path, img)
                print('Image saved!')


        
        


def main():
    cli = CLI()
    cli.interface()
    rospy.spin()


if __name__ == "__main__":
    main()