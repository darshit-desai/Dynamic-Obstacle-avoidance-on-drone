import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

def obstacle_detection(depth_image, focal_length, threshold_oi, threshold_ho):
    # Step 1: Compute U-depth map
    u_depth_map = compute_u_depth_map(depth_image)
    print(u_depth_map)
    # Step 2: Find points of interest
    points_of_interest = find_points_of_interest(u_depth_map, focal_length, threshold_oi)
    print(points_of_interest)
    
    # Step 3: Group points and extract bounding box
    bounding_box = extract_bounding_box(points_of_interest)
    
    # Step 4: Obstacle detection
    x_b_o, y_b_o, l_b_o, w_b_o = calculate_obstacle_parameters(bounding_box, focal_length)
    z_b_o, h_b_o = calculate_obstacle_height(depth_image, bounding_box, focal_length)
    
    return x_b_o, y_b_o, z_b_o, l_b_o, w_b_o, h_b_o

def compute_u_depth_map(depth_image):
    # Compute U-depth map from column depth value histograms
    # Assuming depth_image is a numpy array
    column_histograms = np.sum(depth_image, axis=0)
    max_bin_value = np.max(column_histograms)
    u_depth_map = column_histograms / max_bin_value
    return u_depth_map

def find_points_of_interest(u_depth_map, focal_length, threshold_oi):
    # Find points of interest based on U-depth map and threshold
    points_of_interest = []
    for i, bin_value in enumerate(u_depth_map):
        depth_bin = i
        depth_value = bin_value * focal_length / threshold_oi
        if depth_value > threshold_oi:
            points_of_interest.append((depth_bin, depth_value))
    return points_of_interest

def extract_bounding_box(points_of_interest):
    # Extract bounding box from grouped points of interest
    min_bin = min(points_of_interest, key=lambda x: x[0])[0]
    max_bin = max(points_of_interest, key=lambda x: x[0])[0]
    min_depth = min(points_of_interest, key=lambda x: x[1])[1]
    max_depth = max(points_of_interest, key=lambda x: x[1])[1]
    bounding_box = ((min_bin, min_depth), (max_bin, max_depth))
    return bounding_box

def calculate_obstacle_parameters(bounding_box, focal_length):
    # Calculate obstacle parameters (position and size) in the body frame
    ul, dt = bounding_box[0]
    ur, db = bounding_box[1]
    x_b_o = db
    y_b_o = (ul + ur) * db / (2 * focal_length)
    l_b_o = 2 * (db - dt)
    w_b_o = (ur - ul) * db / focal_length
    return x_b_o, y_b_o, l_b_o, w_b_o

def calculate_obstacle_height(depth_image, bounding_box, focal_length):
    # Calculate obstacle height in the body frame
    ul, dt = bounding_box[0]
    ur, db = bounding_box[1]
    ht = np.max(depth_image[:, ul:ur+1])
    hb = np.min(depth_image[:, ul:ur+1])
    z_b_o = (ht + hb) * db / (2 * focal_length)
    h_b_o = (ht - hb) * db / focal_length
    return z_b_o, h_b_o

img = cv.imread('first_depth_image.png', 0)
focal_length = 684
threshold_oi = 500
threshold_ho = 1800
x_b_o, y_b_o, z_b_o, l_b_o, w_b_o, h_b_o = obstacle_detection(img, focal_length, threshold_oi, threshold_ho)
print(x_b_o, y_b_o, z_b_o, l_b_o, w_b_o, h_b_o)