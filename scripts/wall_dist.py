#!/usr/bin/env python

# ME 7785 - Lab 3
# Team Mazumdar: Noah Kohls, Raymond Kim, Samuel Deal, Hogan Welch, Justine Powell
# lidar view command: rosrun rviz rviz -d ~/catkin_ws/src/turtlebot3/turtlebot3_description/rviz/model.rviz 

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np


class PublishObjectRange:
    def __init__(self):
        self.angle = Float32().data
        self.scan = LaserScan()
        self.pos = Point()
        self.cam_sub = rospy.Subscriber('/angle', Float32, self.set_angle, queue_size=1, buff_size=2**24)
        self.lds_sub = rospy.Subscriber('/scan', LaserScan, self.set_scan, queue_size=2, buff_size=2**24)
        self.pos_pub = rospy.Publisher('/object_pos', Point, queue_size=1)
        self.publish_object_range()

    def set_angle(self, angle):
        self.angle = angle.data

    def set_scan(self, scan):
        self.scan = scan

    def get_object_range(self):
        try:
            if self.angle != 99999:
                if int(round(self.angle)) < 0:
                    obj_idx = int(round(self.angle)) + 360
                else:
                    obj_idx = int(round(self.angle))
                obj_idxs = np.array(range(obj_idx - 5, obj_idx + 5))
                dists = np.array(self.scan.ranges)

                obj_dists = dists[obj_idxs]
                delta = 0.01  # Filter out distances that differ from the min by more than this (in m).
                obj_dists = np.delete(obj_dists, np.argwhere(obj_dists > min(obj_dists) + delta))
                if np.mean(obj_dists) >= 0.02 and np.mean(obj_dists) <= 5.0:
                    self.pos.x = np.mean(obj_dists)
                else:
                    self.pos.x = -1
                self.pos.y = self.angle
                
                wall_idxs = range(4) + range(356, 360)
                wall_dists = dists[wall_idxs]
                wall_dists = np.delete(wall_dists, np.argwhere(obj_dists < 0.02))
                wall_dists = np.hstack([wall_dists, 3.5])
                self.pos.z = min(wall_dists)
            else:
                self.pos.x = -1
                self.pos.y = 0
            
        except IndexError:
            dists = np.zeros(360)
            pass

        
    def publish_object_range(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.get_object_range()
            self.pos_pub.publish(self.pos)
            rate.sleep()


def main():
    rospy.init_node('getObjectRange', anonymous=True)
    try:
        publish_object_range = PublishObjectRange()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()