#!/usr/bin/env python  
import roslib
import rosbag
import rospy
import math
import tf
import sys
import numpy as np
from std_msgs.msg import Float32,Header
from sensor_msgs.msg import CameraInfo, Image,CompressedImage
from gazebo_msgs.srv import SpawnModel, DeleteModel,SetModelState
from gazebo_msgs.msg import ModelState,ModelStates
from cv_bridge import CvBridge
import cv2
import getopt
import matplotlib.pyplot as plt
import os

# Set your directories
root_direc = 'gazebo_kitti'
day_direc ='2024_08_12'
bag_list = ['lakepark1', 'lakepark2', 'lakepark3', 'lakepark4']
sat_direc = 'satmap'
image_direc = ['gt', 'fake1', 'fake2', 'fake3', 'fake4']
noise = [(0, 0), (3, 3), (-3, -3), (3, -3), (-3, 3)]

# Set what to save
is_satcam = True

im_left_check=False
pose_check=False

# img_msg_left=CompressedImage()
img_msg_left=Image()

lidar_topic_name = '/os0_cloud_node/points'
gps_topic_name = '/navsat/fix'

def left_img_cb(msg):
    global img_msg_left
    img_msg_left=msg
    im_left_check=True

def pose_cb(msg):
    global pose_check
    pose_check=True

def get_index_offset(bag, lidar_topic_name, gps_topic_name):
    input_bag = os.path.join(root_direc, 'bag', day_direc, bag + '.bag')
    lidar_index = 0
    gps_index = 0

    with rosbag.Bag(input_bag, 'r') as input_bag:
        for _, _, t in input_bag.read_messages(topics=[lidar_topic_name]):
            lidar_index += 1
        for _, _, t in input_bag.read_messages(topics=[gps_topic_name]):
            gps_index += 1

    index_offset = lidar_index - gps_index
    print("{} index offset for {}".format(index_offset, bag))
    return index_offset

def get_image(image, image_direc, index, lat, long):
    image_cv = bridge.imgmsg_to_cv2(image, "bgr8")
    output_dir = os.path.join(root_direc, sat_direc, day_direc, bag, image_direc)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    image_file = os.path.join(output_dir, "satellite_{:010d}_lat_{}_long_{}.png".format(index, lat, long))
    cv2.imwrite(image_file, image_cv)
    # print("{:d} frames of Image from {} saved".format(index, bag))

if __name__ == '__main__':
    # global img_msg_left
    # global img_msg_right
    # global im_right_check
    # global pose_check

    for i in range(len(image_direc)):
        for bag in bag_list:
            input_bag = os.path.join(root_direc, 'bag', day_direc, bag + '.bag')
            index_offset = get_index_offset(bag, lidar_topic_name, gps_topic_name)

            rospy.init_node('im_listener')
            # rospy.Subscriber("/husky/camera/left/image_raw/compressed",CompressedImage,left_img_cb)
            rospy.Subscriber("/husky/camera/left/image_raw",Image,left_img_cb)
            bridge = CvBridge()
            t_prev=rospy.Time()

            index = min(0, index_offset)
            is_model_state = False
            is_navsat_fix = False
            msg_move = None
            test_msg = None

            for topic, msg, t in rosbag.Bag(input_bag).read_messages():
                if topic== "/navsat/fix":
                    is_navsat_fix = True
                    lat, long = msg.latitude, msg.longitude
                if topic == "/gazebo/model_states":
                    # t_prev=t
                    msg_move=ModelState()
                    msg_move.model_name='stereo_camera'
                    msg_move.pose=msg.pose[len(msg.name)-1]
                    msg_move.pose.orientation.x = 0.0
                    msg_move.pose.orientation.y = 0.7071068
                    msg_move.pose.orientation.z = 0.0
                    msg_move.pose.orientation.w = 0.7071068
                    msg_move.pose.position.x=msg.pose[len(msg.name)-1].position.x + 0.0812 + noise[i][0]
                    msg_move.pose.position.y=msg.pose[len(msg.name)-1].position.y + noise[i][1]
                    msg_move.pose.position.z=40
                    
                    rospy.wait_for_service('/gazebo/set_model_state')
                    try:
                        set_state = rospy.ServiceProxy(
                            '/gazebo/set_model_state', SetModelState)
                        resp = set_state(msg_move)

                    except rospy.ServiceException as e:
                        print ("Service call failed: %s" % e)

                    rospy.sleep(0.01)
                    # print(resp)

                    im_left_check=False
                    pose_check=False
                    
                    if is_navsat_fix:
                        if is_satcam and index >= 0:
                            get_image(img_msg_left, image_direc[i], index, lat, long)
                            print("Image saved: {}, {}, {:010d}".format(bag, image_direc[i], index))
                        index += 1
                        is_navsat_fix = False
            
            print("Image for {}_{} saved".format(bag, image_direc[i]))
        print("{}_image complete".format(image_direc[i]))       
    print("All images saved")