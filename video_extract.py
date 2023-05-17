import rosbag
import cv2
import time
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pylab as plt
bag_file = '/home/darshit/rosbags/enae788m/2023-05-16-16-40-58.bag'  # Replace with the path to your ROS bag file
topic = '/camera/depth/image_rect_raw'  # Replace with the specific image topic you want to process
max_messages = 500  # Number of messages to process

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
    
    first_depth_image = depth_images[600]
    plt.imshow(first_depth_image, cmap='gray')
    plt.show()
    # You can add additional processing logic here after the imshow
count =0 
strt = time.time()
for depth_image in depth_images:

    # Extract the depth values from the image
    depth_values = depth_image.flatten()
    depth_values = depth_values[depth_values != 0]

    bin_size=200
    # Compute the column-wise depth value histograms
    # Compute the column-wise depth value histograms
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
    cv2.rectangle(converted_image, (x, y), (x + width, y + height), (255, 0, 0), 1)

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
        print(z_o_body*15)

        height_o_body = (-h_t+h_b)*d_b*15 / focal_length
        if(z_o_body*15 < 1200):
            cv2.rectangle(converted_image_depth, (y_min, x_min), (y_max, x_max), (255, 0, 0), 2)
        cv2.imshow('Depth Image', converted_image_depth)
        print("frame count: ", count)
        count += 1
        # Check for the 'q' key to break the loop and stop the script
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
cv2.destroyAllWindows()
end = time.time()
elapsed_time = end - strt
print(end)
print(strt)
print("time taken: ", elapsed_time)












