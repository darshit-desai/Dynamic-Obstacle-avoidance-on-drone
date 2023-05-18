"""@package depth_processor
This module processes the depth image and publishes the obstacle detection image
and the UMaps image.
@file depth_processor.py
@brief This module processes the depth image and publishes the obstacle detection image
@date 07/12/2021
@author: Darshit Desai
@maintainer: Darshit Desai
@author: Shivam Sehgal
@maintainer: Shivam Sehgal
"""

import cv2
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped,Pose,Point
import os
os.environ['ROS_PYTHON_VERSION'] = '3'  # Replace '3' with your desired OpenCV version
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import time
from scipy.spatial.transform import Rotation
# Create a rotation object for rotation around the z-axis
import numpy as np
import copy as copy

fps = 0 # frame per second
class ImageListener:
	"""@brief Class to process the depth image and publish the obstacle detection image
	and the UMaps image.
	@details This class subscribes to the depth image topic and the color image topic.
	It processes the depth image and publishes the obstacle detection image and the UMaps image.
	"""
	def __init__(self, topic1, topic2):
		"""@brief Constructor for ImageListener class
		@param topic1 The topic name for the depth image
		@param topic2 The topic name for the color image
		@return None"""
		self.topic1 = topic1
		self.topic2 = topic2
		self.color_image = None
		self.bridge = CvBridge()
		self.sub1 = rospy.Subscriber(topic1, msg_Image, self.imageDepthCallback1,queue_size=10)
		self.sub2 = rospy.Subscriber(topic2, msg_Image, self.imageCallback2,queue_size=10)
		self.pub = rospy.Publisher('/obstacle_detections', msg_Image, queue_size=10)
		self.pub2 = rospy.Publisher('/UMaps', msg_Image, queue_size=10)
		########### PUBLISH BODY POSITION
		# publisher to publish postion of obstacle
		self.pub3=rospy.Publisher('/obstacle_pose',PoseStamped,queue_size=1)
		# Store position msgs
		self.obstacle_pos_body=Point()
		self.obstacle_pos_body.x=0
		self.obstacle_pos_body.y=0
		self.obstacle_pos_body.z=0
		self.pose_stamped_msg = PoseStamped()
		self.pose_stamped_msg.header = Header()
		self.pose_stamped_msg.header.stamp = rospy.Time.now()
		self.pose_stamped_msg.header.frame_id = "map"

		pose_msg=Pose()
		pose_msg.position=self.obstacle_pos_body
		self.pose_stamped_msg.pose=pose_msg

		self.Umaps = None
		self.obstacle_detections = None
	def imageDepthCallback1(self, data):
		"""@brief Callback function for the depth image
		@param data The depth image
		@return None"""
		cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
		self.imagePublisher(cv_image)
	def imageCallback2(self, data):
		"""@brief Callback function for the color image
		@param data The color image
		@return None"""
		self.color_image = self.bridge.imgmsg_to_cv2(data, data.encoding)    
	def imagePublisher(self,cv_image):
		"""@brief Function to process the depth image and publish the obstacle detection image
		and the UMaps image.
		@param cv_image The depth image
		@return None"""
		strt = time.time() # start time
		
		depth_image = copy.deepcopy(cv_image) # copy the depth image

		bin_size=200 # bin size for the histogram
		num_columns = depth_image.shape[1] # number of columns in the depth image
		histograms = np.zeros((bin_size, 1), dtype=np.int16) # initialize the histogram
		# loop through each column in the depth image
		for col in range(num_columns):
			column_values = depth_image[:, col]
			histogram, _ = np.histogram(column_values, bins=bin_size, range=(0,3000))
			histograms = np.column_stack((histograms, histogram))

		focal_length = 382.681243896484 # focal length of the camera in mm
		T_poi = 500 # threshold for the obstacle detection
		T_tho = 1800 # threshold for the UMaps
		values = list(range(0, 3001, 15))  # Generate the list of values spaced in 15 divisions from 0 to 3000
		averages = []
		# loop through the values list
		for i in range(len(values)-1):
			start = values[i]
			end = values[i+1]
			average = (start + end) / 2
			averages.append(average)

		# convert the averages list to a numpy array
		dbin = np.array(averages)
		# calculate the distance of each bin
		T_pois = focal_length * T_tho / (dbin)
		# convert the T_pois list to a numpy array
		res=np.array(T_pois)
		final_arr = np.array(res>T_poi)
		indices = np.where(final_arr == True) # loop through the indices list
		normalized_image_UDepth = cv2.normalize(histograms, None, 0, 255, cv2.NORM_MINMAX) # normalize the histogram
		converted_image_UDepth = np.uint8(normalized_image_UDepth) # convert the normalized image to uint8
		_, binary_image = cv2.threshold(converted_image_UDepth, 15, 36, cv2.THRESH_BINARY) # threshold the image

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

		u_l, d_t = x, y # upper left and top
		u_r, d_b = x + width, y + height # upper right and bottom

		x_o_body = d_b # x_o_body is the distance of the obstacle from the camera
		columns_image_ctr = (depth_image.shape[1])/2 # center of the image
		rows_image_ctr = (depth_image.shape[0])/2 # center of the image
		y_o_body = ((u_l-columns_image_ctr)+(u_r-columns_image_ctr))*d_b/(2*focal_length) # y_o_body is the distance of the obstacle from the camera
		# y_o_body = ((u_l + u_r)*d_b) / (2*focal_length)
		l_o_body = 2*(d_b-d_t) # l_o_body is the length of the obstacle
		w_o_body = (u_r - u_l)*d_b / focal_length # w_o_body is the width of the obstacle
	
		coord_list=[] # list to store the coordinates of the obstacle
		if (u_l>=depth_image.shape[1] or u_r>=depth_image.shape[1]):
			print("out of bounds")
		else:
			listulur =np.linspace(u_l,u_r,20,dtype=int)
			for j in listulur:
				for i in range(depth_image.shape[0]):
					if ((depth_image[i][j] > (d_t*15)) and (depth_image[i][j] < ((d_t+l_o_body)*15))):
						coord_list.append([i,j])

		coordinates = np.array(coord_list)
		# x_max, y_max, x_min, y_min = np.max(coordinates[:,0]), np.max(coordinates[:,1]), np.min(coordinates[:,0]), np.min(coordinates[:,1])
		if len(coordinates) > 0:
			x_max, y_max = np.max(coordinates, axis=0)
			x_min, y_min = np.min(coordinates, axis=0)
		else:
    		# Handle the case when coordinates are empty
			x_max, y_max, x_min, y_min = 0, 0, 0, 0

		#normalize and convert a depth image which is of 64 bit to 8 bit
		normalized_image_depth = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
		converted_image_depth = np.uint8(normalized_image_depth)
		h_t = x_min
		h_b = x_max
		# z_o_body = (h_t+h_b)*d_b / (2*focal_length)
		z_o_body = ((h_t-rows_image_ctr)+(h_b-rows_image_ctr))/(2*focal_length)


		height_o_body = (-h_t+h_b)*d_b*15 / focal_length
		# color_image = self.color_image
		# if(x_o_body*15 < 1500):
		cv2.rectangle(converted_image_depth, (y_min, x_min), (y_max, x_max), (255, 0, 0), 2)
		# Define the font properties
		font = cv2.FONT_HERSHEY_SIMPLEX
		font_scale = 0.5
		color = (255, 0, 0)  # Text color in BGR format
		thickness = 1  # Thickness of the text
		global fps
		text = "ZPose_body: " + str(x_o_body*15) + " mm" + "FPS: " + str(fps) + "FPS"

		# Get the dimensions of the image
		image_height, image_width = converted_image_depth.shape[:2]

		# Calculate the size of the text
		(text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)

		# Calculate the position of the text (top right corner)
		text_position = (image_width - text_width - 10, text_height + 10)

		# Add the text to the image
		cv2.putText(converted_image_depth, text, text_position, font, font_scale, color, thickness)
		self.obstacle_detections = converted_image_depth
		self.Umaps = converted_image_UDepth
		end_t = time.time()
		global frame_count
		frame_count+=1
		fps = 1/(end_t-strt)
		print("Frame count:", frame_count)
		print("Time taken: ", end_t-strt)
		# Apply the rotation to the vector
		rotation_x = Rotation.from_euler('x',180 , degrees=True)
		rotation_z = Rotation.from_euler('z',90 , degrees=True)
		x=(x_o_body*15)/1000
		y=(y_o_body*15)/1000
		z=(z_o_body*15)/1000
		rotated_vector = rotation_x.apply(np.array([x,y,z]))
		rotated_vector = rotation_z.apply(rotated_vector)
		self.obstacle_pos_body.x=rotated_vector[0]
		self.obstacle_pos_body.y=rotated_vector[1]
		self.obstacle_pos_body.z=rotated_vector[2]

frame_count = 0
if __name__ == '__main__':
	"""@brief: This is the main function which initializes the node and creates an object of the class ImageListener
	@params: None
	@returns: None"""
	rospy.init_node("depth_image_processor")
	# topic1 = '/camera/depth/image_rect_raw'  # check the depth image topic in your Gazebo environmemt and replace this with your
	topic1 = '/camera/aligned_depth_to_color/image_raw' 
	topic2  = '/camera/color/image_raw' 
	listener = ImageListener(topic1, topic2)
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		#@brief: This is the main loop which runs the callback function
		obstacleframe = listener.obstacle_detections
		Umapframe = listener.Umaps
		if (obstacleframe is not None or Umapframe is not None):
			#@brief: This is the publisher which publishes the obstacle image and the Umap image
			# modified_msg = listener.bridge.cv2_to_imgmsg(obstacleframe, encoding='mono8')
			# modified_msg.header = Header(stamp=rospy.Time.now())
			# listener.pub.publish(modified_msg)
			# modified_msg2 = listener.bridge.cv2_to_imgmsg(Umapframe, encoding='mono8')
			# modified_msg2.header = Header(stamp=rospy.Time.now())
			# listener.pub2.publish(modified_msg2)

			#@brief: This is the publisher which publishes the pose of the obstacle in the body frame
			listener.pub3.publish(listener.pose_stamped_msg)
			cv2.imshow("obstacle", obstacleframe)
			cv2.imshow("Umap", Umapframe)
			cv2.waitKey(1)
			rate.sleep()
	cv2.destroyAllWindows()
	rospy.spin()
