#!/usr/bin/env python3

import cv2
import sys
import csv
import time
import numpy as np


def crop(original_img):
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

        crop = original_img[ymin[0]:ymax[0], xmin[0]:xmax[0]]

        if cv2.contourArea(largest_item) < 3500:
            test_img = np.array(cv2.resize(img, (33, 25)))
        else:
            test_img = np.array(cv2.resize(crop, (33, 25)))
    else:
        test_img = np.array(cv2.resize(img, (33, 25)))

    return cv2.cvtColor(test_img, cv2.COLOR_RGB2GRAY)


### Load training images and labels

imageDirectory = '/home/burger/catkin_ws/src/team_mazumdar_final_project/2020imgs/'

with open(imageDirectory + 'train.txt', 'r') as f:
    reader = csv.reader(f)
    lines = list(reader)

# this line reads in all images listed in the file in GRAYSCALE, and resizes them to 33x25 pixels
train = np.array([crop(cv2.imread(imageDirectory + lines[i][0] + ".png")) for i in range(len(lines))])

# here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
train_data = train.flatten().reshape(len(lines), 33 * 25)
train_data = train_data.astype(np.float32)

# read in training labels
train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])

### Train classifier
knn = cv2.ml.KNearest_create()
knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

'''if (__debug__):
    Title_images = 'Original Image'
    Title_resized = 'Image Resized'
    cv2.namedWindow(Title_images, cv2.WINDOW_AUTOSIZE)'''

### Run test images
with open(imageDirectory + 'test.txt', 'r') as f:
    reader = csv.reader(f)
    lines = list(reader)

correct = 0.0
confusion_matrix = np.zeros((6, 6))

for i in range(len(lines)):
    original_img = cv2.imread(imageDirectory + lines[i][0] + ".png", cv2.IMREAD_COLOR)
    # test_img = np.array(cv2.resize(cv2.imread(imageDirectory+lines[i][0]+".png",0),(33,25)))
    test_img = crop(original_img)

    '''if (__debug__):
        cv2.imshow(Title_images, original_img)
        cv2.imshow(Title_resized, test_img)
        k = cv2.waitKey()
        if k == 27:  # Esc key to stop
            break'''
    test_img = test_img.flatten().reshape(1, 33 * 25)
    test_img = test_img.astype(np.float32)

    test_label = np.int32(lines[i][1])

    ret, results, neighbours, dist = knn.findNearest(test_img, 3)

    if test_label == ret:
        print(str(lines[i][0]) + " Correct, " + str(ret))
        correct += 1
        confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
    else:
        confusion_matrix[test_label][np.int32(ret)] += 1

        print(str(lines[i][0]) + " Wrong, " + str(test_label) + " classified as " + str(ret))
        print("\tneighbours: " + str(neighbours))
        print("\tdistances: " + str(dist))

print("\n\nTotal accuracy: " + str(correct / len(lines)))
print(confusion_matrix)

knn.save('/home/burger/catkin_ws/src/team_mazumdar_final_project/knn_sign.cv2')

knn.read('/home/burger/catkin_ws/src/team_mazumdar_final_project/knn_sign.cv2')