import rosbag
import cv2
from cv_bridge import CvBridge

bag_file = '/home/darshit/rosbags/enae788m/2023-05-16-05-09-08.bag'  # Replace with the path to your ROS bag file
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
    first_depth_image = depth_images[-1200]
    cv2.imshow("First Depth Image", first_depth_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # You can add additional processing logic here after the imshow