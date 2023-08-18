#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class Converter():
    def __init__(self):
        rospy.init_node("acc_to_vel")
        self.sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.pub = rospy.Publisher("/est_velocity", Twist, queue_size=10)

        self.vel_msg = Twist()
        self.ax = 0.0; self.ay = 0.0; self.az = 0.0             # linear acceleration
        self.awx = 0.0; self.awy = 0.0; self.awz = 0.0          # angular acceleration

        self.lx = 0.0; self.ly = 0.0; self.lz = 0.0             # linear velocity
        self.wx = 0.0; self.wy = 0.0; self.wz = 0.0             # angular velocity

        self.dt = 1.0/400.0
            
    def imu_callback(self, msg):
        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.az = msg.linear_acceleration.z

        self.awx = msg.angular_velocity.x
        self.awy = msg.angular_velocity.y
        self.awz = msg.angular_velocity.z


    def convert(self):
        self.lx += self.ax * self.dt
        self.ly += self.ay * self.dt
        self.lz += self.az * self.dt
        # self.wx += self.awx * self.dt
        # self.wy += self.awy * self.dt
        # self.wz += self.awz * self.dt

        print("Linear velocity: {}, {}, {}".format(self.lx, self.ly, self.lz))
        print("Angular velocity: {}, {}, {}".format(self.awx, self.awy, self.awz))

        self.vel_msg.linear.x = self.lx
        self.vel_msg.linear.y = self.ly
        self.vel_msg.linear.z = self.lz
        self.vel_msg.angular.x = self.awx
        self.vel_msg.angular.y = self.awy
        self.vel_msg.angular.z = self.awz
        self.pub.publish(self.vel_msg)


def main():
    conv = Converter()

    while not rospy.is_shutdown():
        conv.convert()
        rospy.sleep(1)


if __name__ == "__main__":
    main()