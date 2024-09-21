#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


class Drive_Wheels():
    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.twist = Twist()
        self.drive_wheels()

    def drive_wheels(self):
        rate = rospy.Rate(10)
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.twist)
            rate.sleep()

def main():
  rospy.init_node('drive_wheels')
  try:
      drive_wheels = Drive_Wheels()
  except rospy.ROSInterruptException:
      pass

if __name__ == '__main__':
    main()