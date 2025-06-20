#!/usr/bin/env python3
import rospy
import cv2
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import copy
import math

#from global_constants import WINDOW_X, WINDOW_Y

WINDOW_X = 90
WINDOW_Y = 30

def rotate_and_center_on_point(image, cx, cy, angle_radians):
    angle_degrees = np.degrees(angle_radians)
    
    height, width = image.shape[:2]
    
    display_center_x = width // 2
    display_center_y = height // 2
    
    rotation_matrix = cv2.getRotationMatrix2D((cx, cy), angle_degrees, 1.0)
    
    #After rotation, (cx, cy) is still at (cx, cy), so we need to translate by:
    translation_x = display_center_x - cx
    translation_y = display_center_y - cy
    
    rotation_matrix[0, 2] += translation_x
    rotation_matrix[1, 2] += translation_y
    
    # Apply the combined transformation
    result_image = cv2.warpAffine(image, rotation_matrix, (width, height))
    
    return result_image, rotation_matrix

def cropped_rotated_window(image, cx, cy, angle_radians,
                           window_x=WINDOW_X, window_y=WINDOW_Y):
    """
    Crop an enlarged patch around (cx,cy), rotate it by angle_radians,
    then extract the central window_x×window_y region.
    Returns: window (window_y×window_x BGR image)
    """
    HEIGHT, WIDTH = image.shape[:2]
    hw, hh = window_x//2, window_y//2
    # enlarge radius to cover rotation
    D = int(math.ceil(math.hypot(hw, hh)))


    x1 = max(0, cx - D)
    y1 = max(0, cy - D)
    x2 = min(WIDTH, cx + D)
    y2 = min(HEIGHT, cy + D)
    patch = image[y1:y2, x1:x2]
    ph, pw = patch.shape[:2]
    center = (pw/2, ph/2)
    angle_deg = -angle_radians * 180.0/math.pi
    M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    rotated = cv2.warpAffine(
        patch, M, (pw, ph),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0,0,0)
    )
    cx_p, cy_p = pw//2, ph//2
    win = rotated[cy_p-hh:cy_p+hh, cx_p-hw:cx_p+hw]
    return win


def callback(data):
    x = int(data.x)
    y = int(data.y)
    map_copy = copy.copy(map_image)
    
    # rotated_map ,_ = rotate_and_center_on_point(map_copy,x,y,data.theta + np.pi / 2)
    
    # angle_deg = np.degrees(data.theta)
    # M = cv2.getRotationMatrix2D(center=(x, y),
    #                         angle=angle_deg,
    #                         scale=1.0)

    # rotated_map = cv2.warpAffine(map_copy, M,
    #                          dsize=(WIDTH, HEIGHT),
    #                          flags=cv2.INTER_LINEAR,
    #                          borderMode=cv2.BORDER_CONSTANT,
    #                          borderValue=(0,0,0))


    # Clamp window boundaries to avoid going out of image bounds
    # x1 = max(0, x - WINDOW_X // 2)
    # y1 = max(0, y - WINDOW_Y // 2)
    # x2 = min(WIDTH, x1 + WINDOW_X)
    # y2 = min(HEIGHT, y1 + WINDOW_Y)

    # # Adjust if near the edge
    # if x2 - x1 < WINDOW_X:
    #     x1 = max(0, x2 - WINDOW_X)
    # if y2 - y1 < WINDOW_Y:
    #     y1 = max(0, y2 - WINDOW_Y)

    # Draw a rectangle on a copy of the map for visualization
    #map_copy = map_image.copy()
    # x1 = WIDTH//2 - WINDOW_X//2
    # y1 = HEIGHT//2 - WINDOW_Y//2
    # x2 = WIDTH//2 + WINDOW_X//2
    # y2 = HEIGHT//2 + WINDOW_Y//2
    # window = rotated_map[y1:y2, x1:x2]
    # cv2.rectangle(rotated_map, (x1,y1),  (x2,y2),(0, 0, 255), 3)
    # cv2.imshow('Map View', rotated_map)

   
    window = cropped_rotated_window(map_copy,x,y,data.theta - np.pi)
    cv2.imshow('window',window)
    cv2.waitKey(1)
    out_msg = bridge.cv2_to_imgmsg(window, encoding='bgr8')
    pub.publish(out_msg)

if __name__ == '__main__':
    map_image = cv2.imread('route.png', cv2.IMREAD_COLOR)
    if map_image is None:
        rospy.logerr("Failed to load map image: route.png")
        exit(1)

    HEIGHT, WIDTH, _ = map_image.shape
    bridge = CvBridge()
    pub = None

    rospy.init_node('window_node', anonymous=True)
    pub = rospy.Publisher('/sim/window', Image, queue_size=1)
    rospy.Subscriber('/sim/pose', Pose2D, callback)
    rospy.loginfo("Window node started. Waiting for /sim/pose messages...")
    rospy.spin()
