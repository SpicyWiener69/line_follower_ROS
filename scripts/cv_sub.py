#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
from geometry_msgs.msg import Twist
import math
# Window size (camera view) in pixels

WINDOW_X = 90
WINDOW_Y = 30

# Toggle on-screen debugging windows
DEBUG = True

def compute_error_from_window(window):
    """
    Given a BGR window of size WINDOW_Y×WINDOW_X, threshold and
    find the largest contour, then compute horizontal error from center.
    Returns (error, cx) where error = cx - (WINDOW_X // 2).
    """
    gray = cv2.cvtColor(window, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, None

    largest = max(contours, key=cv2.contourArea)

    if DEBUG:
        dbg = window.copy()
        cv2.drawContours(dbg, [largest], -1, (0,255,0), 2)
        cv2.imshow('Detected Contour', dbg)
        cv2.waitKey(1)
    M = cv2.moments(largest)
    if M['m00'] == 0:
        return 0, None

    cx = int(M['m10'] / M['m00'])
    lateral_error = cx - (WINDOW_X // 2)

    [vx, vy, x0, y0] = cv2.fitLine(largest, cv2.DIST_L2, 0, 0.01, 0.01)
    line_angle = math.atan2(vy, vx)
    line_angle = line_angle + math.pi/2
    if line_angle > math.pi /2:
        line_angle -= math.pi
    # Normalize to [-π, π]
    #line_angle = math.atan2(math.sin(line_angle), math.cos(line_angle))
    # If “forward” is up the image (negative y), we want
    #orientation_error = line_angle - (−π/2) = line_angle + π/2
    #orientation_error = normalize_angle(line_angle + math.pi/2)
    return lateral_error, line_angle

def normalize_angle(a):
    #[−pi, pi]
    return (a + math.pi) % (2*math.pi) - math.pi

def P_controller(lateral_error,orientation_error, kp_lat = -0.007, kp_or = -0.4, speed = 20):
    msg = Twist()

    omega = (kp_lat * lateral_error) + (kp_or * orientation_error)
    #omega = (lateral_error * kp_lat)
    #omega = (kp_or * orientation_error)
    msg.angular.z = omega
    msg.linear.x = speed
    return msg

def callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    h, w = frame.shape[:2]
    if (w, h) != (WINDOW_X, WINDOW_Y):
        rospy.logwarn(f"Expected {WINDOW_X}×{WINDOW_Y}, got {w}×{h}")

    if DEBUG:
        cv2.imshow('Camera Window', frame)
        cv2.waitKey(1)

    lateral_error, orientation_error = compute_error_from_window(frame)
    rospy.loginfo(f"lat Error: {lateral_error} or error:{orientation_error}rad")
    msg = P_controller(lateral_error, orientation_error)
    print(f'publishing: angular:{msg.angular.z} speed:{msg.linear.x}')
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('cv_sub', anonymous=True)
    bridge = CvBridge()
    rospy.Subscriber('/sim/window', Image, callback)
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=4)
    rospy.loginfo("cv_sub node ready, listening on /sim/window")
    rospy.spin()
    cv2.destroyAllWindows()