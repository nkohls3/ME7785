#!/usr/bin/env python

# ME 7785 - Lab 1
# Team 1: Noah Kohls, Raymond S Kim, Samuel James Deal, Hogan Matthew Welch, Justine Clair Powell


import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
import numpy as np
import cv2
import sys
import csv
import time
from cv_bridge import CvBridge, CvBridgeError

class Classify_Object():
	def __init__(self):
		self.knn = self.train()
		self.bridge = CvBridge()
		self.cmd_pub = rospy.Publisher('sign',Point,queue_size=1)
		#self.img_pub = rospy.Publisher('debug_img',CompressedImage,queue_size=1)
		self.img_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.get_image,queue_size=1,buff_size=2**24)
		self.point = Point()
		#self.debug_image = CompressedImage()
		self.classify_object()

	def get_image(self, image):
		img = self.bridge.compressed_imgmsg_to_cv2(image, 'bgr8')
		test_img = self.crop(img)
		#self.debug_image = self.bridge.cv2_to_compressed_imgmsg(test_img)
		test_img = test_img.flatten().reshape(1, 33 * 25)
		test_img = test_img.astype(np.float32)
		ret, results, neighbours, dist = self.knn.findNearest(test_img, 3)
		self.point.x = ret

	def crop(self, original_img):
		# Gaussian Blur Image
		img = cv2.medianBlur(original_img, 5)

		# Convert to HSV (hue, saturation, intensity)
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		# Color Mask
		lower_red = np.array([173, 80, 50])
		upper_red = np.array([179, 255, 200])
		# Make a mask to show only allowable color from image
		red_mask1 = cv2.inRange(hsv, lower_red, upper_red)
		# Color Mask
		lower_red = np.array([0, 80, 50])
		upper_red = np.array([10, 255, 200])
		# Make a mask to show only allowable color from image
		red_mask2 = cv2.inRange(hsv, lower_red, upper_red)
		red_mask = red_mask1 + red_mask2
		#cv2.imshow('RM', red_mask)#

		# Color Mask1
		lower_blue = np.array([100, 50, 30])
		upper_blue = np.array([130, 255, 255])

		# Make a mask to show only allowable color from image
		blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
		#cv2.imshow('BM', blue_mask)

		# Color Mask1

		lower_green = np.array([50, 50, 20])
		upper_green = np.array([80, 255, 255])

		# Make a mask to show only allowable color from image
		green_mask = cv2.inRange(hsv, lower_green, upper_green)
		#cv2.imshow('GM', green_mask)

		mask = red_mask + blue_mask + green_mask

		# Mask opening and closing
		kernel2 = np.ones((5, 5), np.uint8)
		mask2 = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel2)
		mask3 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel2)
		holes = mask3.copy()
		cv2.floodFill(holes, None, (0, 0), 255)  # line 27
		holes = cv2.bitwise_not(holes)
		mask_img = cv2.bitwise_or(mask, holes)

		_, contours, _ = cv2.findContours(mask_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

		if len(sorted_contours) > 0:
			largest_item = sorted_contours[0]

			xmin = min(largest_item[:, :, 0])
			xmax = max(largest_item[:, :, 0])
			ymin = min(largest_item[:, :, 1])
			ymax = max(largest_item[:, :, 1])

			crop_img = original_img[ymin[0]:ymax[0], xmin[0]:xmax[0]]

			if cv2.contourArea(largest_item) < 3500:
				test_img = np.array(cv2.resize(img, (33, 25)))
			else:
				test_img = np.array(cv2.resize(crop_img, (33, 25)))

			
		else:
			test_img = np.array(cv2.resize(img, (33, 25)))

		return cv2.cvtColor(test_img, cv2.COLOR_RGB2GRAY)

	def train(self):
		### Load training images and labels

		imageDirectory = '/home/burger/catkin_ws/src/team_mazumdar_final_project/2020imgs/'

		with open(imageDirectory + 'train.txt', 'r') as f:
			reader = csv.reader(f)
			lines = list(reader)

		# this line reads in all images listed in the file in GRAYSCALE, and resizes them to 33x25 pixels
		train = np.array([self.crop(cv2.imread(imageDirectory + lines[i][0] + ".png")) for i in range(len(lines))])

		# here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
		train_data = train.flatten().reshape(len(lines), 33 * 25)
		train_data = train_data.astype(np.float32)

		# read in training labels
		train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])

		### Train classifier
		knn = cv2.ml.KNearest_create()
		knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)
		return knn


	def classify_object(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.cmd_pub.publish(self.point)
			#self.img_pub.publish(self.debug_image)
			rate.sleep()

def main():
	rospy.init_node('classify_object')
	try:
		classify_object = Classify_Object()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
    main()