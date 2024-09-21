#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class Drive_Wheels():
    def __init__(self):
        self.Init = True
        self.Init_pos = Point()
        self.globalPos = Point()
        self.globalAng = 0
        self.heading = 0
        self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.sign_sub = rospy.Subscriber('sign',Point,self.update_Sign,queue_size=1)
        self.wall_dist_sub = rospy.Subscriber('object_pos',Point,self.update_Wall_Dist,queue_size=1)
        self.odom_sub = rospy.Subscriber('odom',Odometry,self.update_Odometry,queue_size=1)
        self.sign_list = [0]*10
        self.sign_angle_list = [0]*10
        self.current_sign = 0
        self.des_dist = 0.35
        self.desired_angle = 0.0
        self.wall_dist = self.des_dist
        self.command_read = False
        self.command = 0
        self.twist = Twist()
        self.drive_wheels()
        
    def set_speeds(self):
        dist_err = self.wall_dist - self.des_dist
        
        if (abs(dist_err) < 0.05) and not self.command_read:
            if self.current_sign:
                self.command_read = True
                self.command = self.current_sign
                rospy.loginfo("Identified Sign %d", self.command)
                rospy.loginfo("Global Angle %f rad", self.globalAng)
                rospy.loginfo("Dist err %f m", dist_err)

            # Empty Wall
            #if self.command == 0:
                #todo

            # Left Turn
            if self.command == 1:
                #self.desired_angle = self.globalAng - math.pi/2
                self.desired_angle += math.pi/2
                if self.desired_angle > math.pi: self.desired_angle -= 2*math.pi
                rospy.loginfo("Turn to %f rad", self.desired_angle)

            # Right Turn
            elif self.command == 2:
                #self.desired_angle = self.globalAng + math.pi/2
                self.desired_angle -= math.pi/2
                if self.desired_angle < -1*math.pi: self.desired_angle += 2*math.pi
                rospy.loginfo("Turn to %f rad", self.desired_angle)

            # Do not enter
            elif self.command == 3:
                #self.desired_angle = self.globalAng + math.pi
                self.desired_angle += math.pi
                if self.desired_angle >= 2*math.pi: self.desired_angle -= 2*math.pi
                rospy.loginfo("Turn to %f rad", self.desired_angle)

            # Goal
            elif self.command == 4:
                rospy.loginfo("Found Goal!")


        if self.command_read:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            angle_err = self.desired_angle - self.globalAng
            if angle_err < -1*math.pi: angle_err += 2*math.pi
            if angle_err > math.pi: angle_err -= 2*math.pi

            #angle_err = math.atan2(-math.sin(self.desired_angle - self.globalAng), -math.cos(self.desired_angle - self.globalAng))
            

            #if self.command == 0:
                #todo

            if (self.command == 1) or (self.command == 2) or (self.command == 3):
                self.twist.angular.z = 0.5*angle_err
                if abs(angle_err) < 0.05:
                    self.command_read = False
                    # Lock in current heading
                    rospy.loginfo("Global Angle %f rad", self.globalAng)
                    self.heading = self.globalAng

            elif self.command == 4:
                rospy.loginfo("Found Goal!")
            
        else:
            '''
            if self.wall_dist == -1:
                self.twist.linear.x = -0.1
            else:
                dist_err = self.des_dist - self.wall_dist
                if dist_err <= 0:
                    self.twist.linear.x = -0.2*dist_err
                    self.twist.angular.z = 0
                else:
                    self.twist.linear.x = -0.1*dist_err
                    self.twist.angular.z = 0  
                    '''
            self.drive_to_sign()
    
    def drive_to_sign(self):
        if self.wall_dist != -1 and self.wall_dist != 0 and self.wall_dist != 3.5:
            if self.wall_dist > 1.0:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0
            elif self.sign_dist != -1:
                dist_err = self.des_dist - self.sign_dist
                if abs(dist_err) <= 0.05: 
                    dist_err = 0
                angle_err = self.sign_angle
                if abs(angle_err) <= 1: 
                    angle_err = 0
                if dist_err > 0: # too close
                    self.twist.linear.x = -0.1*dist_err
                else: # too far
                    self.twist.linear.x = -0.1*dist_err

                self.twist.angular.z = 0.01*angle_err
            elif self.wall_dist < 0.3:
                self.twist.linear.x = -0.1
                #rospy.loginfo("Angle Error %f rad", self.globalAng)
                #rospy.loginfo("Sign Dist %f rad", self.globalAng)
        #else:
            #self.twist.linear.x =  0.0
            #self.twist.angular.z = 0.0
            #self.command_read = False
            
            

    def update_Odometry(self, Odom):
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z

        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

        self.set_speeds()

    def update_Sign(self, point):
        # Append to array and find the mode
        self.sign_list.append(point.x)
        self.sign_list.pop(0)
        self.current_sign = max(set(self.sign_list), key=self.sign_list.count)

    def update_Wall_Dist(self, point):
        self.wall_dist = point.z
        self.sign_dist = point.x

        self.sign_angle_list.append(point.y)
        self.sign_angle_list.pop(0)
        self.sign_angle = max(set(self.sign_angle_list), key=self.sign_angle_list.count)

    def drive_wheels(self):
        rate = rospy.Rate(10)
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