#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image, NavSatFix
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import getopt
import os
import sys
import hashlib

# Set your directories
root_direc = 'gazebo_kitti'
satmap_direc = 'satmap'
map_name = 'lakepark'
z = 40
stride = 10
global img_msg_left, img_left_check

img_left_check = False
img_msg_left = None
# gps_data = None
img_old = None
attempt = 0
max_attempts = 500
count = 0

# Reference location
REFERENCE_LAT = 49.9
REFERENCE_LON = 8.9

# Meters per degree
METERS_PER_DEGREE_LAT = 111320.0
METERS_PER_DEGREE_LON = 71703.48

def left_img_cb(msg):
    global img_msg_left, img_left_check
    img_msg_left=msg
    img_left_check=True
    # rospy.loginfo("Received image message with encoding: {}".format(msg.encoding))

# def gps_cb(msg):
#     global gps_data
#     gps_data = msg
#     rospy.loginfo("Received GPS data: lat = {}, lon = {}".format(msg.latitude, msg.longitude))

def xy_to_latlon(x, y):
    lat = REFERENCE_LAT + (x / METERS_PER_DEGREE_LON)
    lon = REFERENCE_LON + (-y / METERS_PER_DEGREE_LON)
    return lat, lon

def compute_image_hash(image):
    """Compute the hash of the given image."""
    image_bytes = cv2.imencode('.png', image)[1].tobytes()
    return hashlib.md5(image_bytes).hexdigest()


if __name__ == '__main__':

    bridge = CvBridge()

    output_direc = os.path.join(root_direc, satmap_direc, "{}_{}_{}".format(map_name, z, stride))
    if not os.path.exists(output_direc):
        os.makedirs(output_direc)

    rospy.init_node('im_listener')
    rospy.Subscriber("/husky/camera/left/image_raw",Image,left_img_cb)
    # rospy.Subscriber("/navsat/fix",NavSatFix,gps_cb)
    # rospy.set_param('/use_sim_time', True)  # Ensure simulation time is being used

    # Initial position and orientation of the camera
    initial_pose = ModelState()
    initial_pose.model_name = 'stereo_camera'
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = z
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.7071068
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 0.7071068

    x_range = np.arange(-170, 175, stride)
    y_range = np.arange(-170, 175, stride)

    # # 50 for test
    # x_range = np.arange(-50, 50, 5)
    # y_range = np.arange(-50, 50, 5)

    for x in x_range:
        for y in y_range:
            new_pose = initial_pose
            new_pose.pose.position.x = x
            new_pose.pose.position.y = y
            
            # lat, lon = gps_data.latitude, gps_data.longitude
            lat, lon = xy_to_latlon(x, y)
            img_filename = os.path.join(output_direc, 'satellite_lat_{:.14f}_long_{:.14f}.png'.format(lat, lon))

            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state(new_pose)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                rospy.signal_shutdown("Failed to set model state")

            # rospy.sleep(0.1)  # Wait for the camera to stabilize

            img_msg_left = None
            img_left_check = False

            # attempt = 0
            while not img_left_check and attempt < max_attempts:
                attempt += 1
                rospy.sleep(0.1)
            print("Set model state done, attempt={}".format(attempt))
            attempt = 0

            # if img_left_check and gps_data is not None:
            if img_left_check and img_msg_left is not None:
                try:
                    img = bridge.imgmsg_to_cv2(img_msg_left, "bgr8")
                    while img_old is not None and np.array_equal(img, img_old):
                        count += 1
                        img_msg_left is None
                        img_left_check = False
                        while img_left_check:
                            rospy.sleep(0.1)
                        img = bridge.imgmsg_to_cv2(img_msg_left, "bgr8")
                    print("Image same count: {}".format(count))
                    count = 0

                    cv2.imwrite(img_filename, img)
                    img_old = img
                    rospy.loginfo("\nImage saved: ({}, {})".format(x, y))
                    img_msg_left = None
                    img_left_check = False
                    # gps_data = None
                except Exception as e:
                    rospy.logerr("Failed to save image: %s" % e)
                    rospy.signal_shutdown("Failed to save image")
            else:
                rospy.logerr("Failed to get image or GPS data at position ({}, {})".format(x, y))
                rospy.signal_shutdown("Failed to get image")

    print("All images saved to {}".format(output_direc))