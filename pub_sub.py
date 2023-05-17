import cv2
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
import os
os.environ['ROS_PYTHON_VERSION'] = '3'  # Replace '3' with your desired OpenCV version
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

import numpy as np
import copy as copy

class ImageListener:
	def __init__(self, topic):
		self.topic = topic
		self.bridge = CvBridge()
		self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback,queue_size=1)
		self.pub = rospy.Publisher('/obstacle_detections', msg_Image, queue_size=1)
	def imageDepthCallback(self, data):
		cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
		self.imagePublisher(cv_image)

	def imagePublisher(self,cv_image):
		depth_image = copy.deepcopy(cv_image)
		depth_values = depth_image.flatten()
		depth_values = depth_values[depth_values != 0]

		bin_size=200
		num_columns = depth_image.shape[1]
		histograms = np.zeros((bin_size, 1), dtype=np.int16)

		for col in range(num_columns):
			column_values = depth_image[:, col]
			histogram, _ = np.histogram(column_values, bins=bin_size, range=(0,3000))
			histograms = np.column_stack((histograms, histogram))

		column_depth_values = []
		for col in range(depth_image.shape[1]):
			column_depth_values.extend(depth_image[:, col])

		focal_length = 382.681243896484
		T_poi = 500
		T_tho = 1800
		values = list(range(0, 3001, 15))  # Generate the list of values spaced in 50 divisions from 0 to 3000
		averages = []

		for i in range(len(values)-1):
			start = values[i]
			end = values[i+1]
			average = (start + end) / 2
			averages.append(average)


		dbin = np.array(averages)

		T_pois = focal_length * T_tho / (dbin)

		res=np.array(T_pois)
		final_arr = np.array(res>T_poi)
		indices = np.where(final_arr == True)
		new_hist = histograms[indices[0], :]
		# print(T_pois)

		normalized_image = cv2.normalize(histograms, None, 0, 255, cv2.NORM_MINMAX)
		converted_image = np.uint8(normalized_image)
		# conv_image = cv2.GaussianBlur(converted_image, (5, 5), 0)
		_, binary_image = cv2.threshold(converted_image, 15, 36, cv2.THRESH_BINARY)

		# Find contours in the binary image
		contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		areas = []
		coords = []
		# Iterate through contours and draw bounding boxes
		for contour in contours:
			x, y, width, height = cv2.boundingRect(contour)
			areas.append(cv2.contourArea(contour))
			coords.append((x, y, width, height))

		#max of areas list index gets coords
		max_index = areas.index(max(areas))
		x, y, width, height = coords[max_index]
		# cv2.rectangle(converted_image, (x, y), (x + width, y + height), (255, 0, 0), 1)

		u_l, d_t = x, y
		u_r, d_b = x + width, y + height

		x_o_body = d_b
		y_o_body = ((u_l + u_r)*d_b) / (2*focal_length)
		l_o_body = 2*(d_b-d_t)
		w_o_body = (u_r - u_l)*d_b / focal_length

		coord_list=[]
		if (u_l>=640 or u_r>=640):
			print("out of bounds")
		else:
			for i in range(depth_image.shape[0]):
				for j in range(u_l,u_r+1):
					if ((depth_image[i][j] > (d_t*15)) and (depth_image[i][j] < ((d_t+l_o_body)*15))):
						coord_list.append([i,j])

		coordinates = np.array(coord_list)
		x_max, y_max, x_min, y_min = np.max(coordinates[:,0]), np.max(coordinates[:,1]), np.min(coordinates[:,0]), np.min(coordinates[:,1])

		#normalize and convert a depth image which is of 64 bit to 8 bit
		normalized_image_depth = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
		converted_image_depth = np.uint8(normalized_image_depth)
		h_t = x_min
		h_b = x_max

		z_o_body = (h_t+h_b)*d_b / (2*focal_length)

		height_o_body = (-h_t+h_b)*d_b*15 / focal_length
		if(z_o_body*15 < 1200):
			cv2.rectangle(converted_image_depth, (y_min, x_min), (y_max, x_max), (255, 0, 0), 2)
		
		modified_msg = self.bridge.cv2_to_imgmsg(converted_image_depth, encoding='mono8')
		modified_msg.header = Header(stamp=rospy.Time.now())
		self.pub.publish(modified_msg)      




if __name__ == '__main__':
	rospy.init_node("depth_image_processor")
	topic = '/camera/depth/image_rect_raw'  # check the depth image topic in your Gazebo environmemt and replace this with your
	listener = ImageListener(topic)
	rospy.spin()
