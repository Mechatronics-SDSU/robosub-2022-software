import numpy as np
import cv2
import imutils
from imutils import paths
from PIL import Image
import sys

def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 35, 125)

    # find the contours in the edged image and keep the largest one;
	# we'll assume that this is our piece of paper in the image
	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	c = max(cnts, key = cv2.contourArea)

	# compute the bounding box of the of the paper region and return it
	return cv2.minAreaRect(c)

def distance_to_camera(known_width, focal_length, per_width):
	# compute and return the distance from the maker to the camera
	return (known_width * focal_length) / per_width

if __name__ == "__main__":
    if len(sys.argv) > 1:
        image_path = sys.argv[1].replace(' ', '')
        img = cv2.imread(image_path)
        marker = find_marker(img)

        known_distance = 24.0
        known_width = 11.0

        focalLength = (marker[1][0] * known_distance) / known_width

    else:
        print("Error: not enough arguments. (Was the ahrs dev port passed?)")
        sys.exit(1)
