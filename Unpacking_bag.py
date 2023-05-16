import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pylab as plt
bag_file = '/home/shivam/Dronebag/test.bag'  # Replace with the path to your ROS bag file
topic = '/camera/depth/image_rect_raw'  # Replace with the specific image topic you want to process
# max_messages = 500  # Number of messages to process

# Open the ROS bag file
bag = rosbag.Bag(bag_file)

depth_images = []  # List to store the processed depth images

count = 0  # Counter for processed messages

bridge = CvBridge()

# Iterate over each message in the bag
for _, msg, _ in bag.read_messages(topics=[topic]):
    # Check if the message is of type sensor_msgs/Image
    if msg._type == 'sensor_msgs/Image':
        try:
            # Convert the depth image message to an OpenCV image
            depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Check if the depth image is valid
            if depth_image is not None:
                # Append the depth image to the list
                depth_images.append(depth_image)

                # Increment the counter
                count += 1

                # Check if the maximum number of messages has been reached
                # if count >= max_messages:
                #     break
            else:
                print("Invalid depth image")
        except Exception as e:
            print("Error decoding depth image:", str(e))

# Close the bag file
bag.close()

# Display the first depth image
if depth_images:
    
    first_depth_image = depth_images[700]
    print(first_depth_image)
    cv2.imshow("First Depth Image", first_depth_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # You can add additional processing logic here after the imshow

depth_image=first_depth_image

# Extract the depth values from the image
depth_values = depth_image.flatten()
depth_values = depth_values[depth_values != 0]
# Compute the histogram
hist, bins = np.histogram(depth_values, bins=50, range=(0, 1000))  # Adjust the number of bins and range as needed

# Plot the histogram
plt.bar(bins[:-1], hist, width=1)
plt.xlabel('Depth Value')
plt.ylabel('Frequency')
plt.title('Depth Value Histogram')
plt.show()
bin_size=20
# Compute the column-wise depth value histograms
# Compute the column-wise depth value histograms
num_columns = depth_image.shape[1]
histograms = np.zeros((bin_size, 1), dtype=np.int16)

for col in range(num_columns):
    column_values = depth_image[:, col]
    histogram, _ = np.histogram(column_values, bins=bin_size, range=(0,1000))
    histograms = np.column_stack((histograms, histogram))
    
    
    

# Display the U-depth map
plt.imshow(histograms, cmap='gray')
plt.xlabel('Column')
plt.ylabel('Row')
plt.title('U-Depth Map')
plt.colorbar()
plt.show()