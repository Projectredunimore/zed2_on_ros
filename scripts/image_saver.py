#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import Image


class CLI():
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)

        # get cameras names from launch params
        self.cam1 = "zed2"
        self.cam2 = "d435_left"
        self.cam3 = "d435_right"

        # check services exist
        rospy.wait_for_service('/image_saver/{}/save'.format(self.cam1))
        rospy.wait_for_service('/image_saver/{}/save'.format(self.cam2))
        rospy.wait_for_service('/image_saver/{}/save'.format(self.cam3))

        # create service proxys
        self.srv1 = rospy.ServiceProxy('/image_saver/{}/save'.format(self.cam1), Empty)
        self.srv2 = rospy.ServiceProxy('/image_saver/{}/save'.format(self.cam2), Empty)
        self.srv3 = rospy.ServiceProxy('/image_saver/{}/save'.format(self.cam3), Empty)

        self.empty = Empty()

    
    def interface(self):
        print('Welcome to image saver CLI')
        print('Type the number of the camera you want to save the image from')
        print('1. {}'.format(self.cam1))
        print('2. {}'.format(self.cam2))
        print('3. {}'.format(self.cam3))
        print('0. Exit')

        while not rospy.is_shutdown():
            try:
                cam = int(input('Camera: '))
                if cam == 1:
                    self.srv1(self.empty)
                elif cam == 2:
                    self.srv2(self.empty)
                elif cam == 3:
                    self.srv3(self.empty)
                elif cam == 0:
                    break
                else:
                    print('Invalid option')
            except ValueError:
                print('Invalid option')


        
        


def main():
    cli = CLI()
    cli.interface()
    rospy.spin()


if __name__ == "__main__":
    main()