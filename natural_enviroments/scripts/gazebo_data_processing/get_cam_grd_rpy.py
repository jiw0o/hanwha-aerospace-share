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
grd_direc = 'raw_data'
rpy_direc = 'oxts/data_rpy'
image_direc = 'image_02/data'

# Set what to save
is_rpy = True
is_grdcam = True

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

def quaternion_to_euler(orientation):
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
        )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler  # roll, pitch, yaw

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

def get_rpy(orientation, rpy_direc, index):
    roll, pitch, yaw = quaternion_to_euler(orientation)
    output_dir = os.path.join(root_direc, grd_direc, day_direc, bag, rpy_direc)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    output_file = os.path.join(output_dir,"{:010d}.txt".format(index))
    with open(output_file, 'w') as out_file:
        out_file.write("{:.6f} {:.6f} {:.6f}\n".format(roll, pitch, yaw))
    print("{:d} frames of Orientation data from {} saved".format(index, bag))

def get_image(image, image_direc, index):
    image_cv = bridge.imgmsg_to_cv2(image, "bgr8")
    output_dir = os.path.join(root_direc, grd_direc, day_direc, bag, image_direc)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    image_file = os.path.join(output_dir, "{:010d}.png".format(index))
    cv2.imwrite(image_file, image_cv)
    print("{:d} frames of Image from {} saved".format(index, bag))

if __name__ == '__main__':
    # global img_msg_left
    # global img_msg_right
    # global im_right_check
    # global pose_check

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
            if topic == "/gazebo/model_states":
                # t_prev=t
                msg_move=ModelState()
                msg_move.model_name='stereo_camera'
                msg_move.pose=msg.pose[len(msg.name)-1]
                msg_move.pose.position.x=msg.pose[len(msg.name)-1].position.x+0.0812
                msg_move.pose.position.z=msg.pose[len(msg.name)-1].position.z+0.245+0.52
                orientation = msg_move.pose.orientation
                
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
                    if is_rpy and index >= 0:
                        get_rpy(orientation, rpy_direc, index)
                    if is_grdcam and index >= 0:
                        get_image(img_msg_left, image_direc, index)
                    index += 1
                    is_navsat_fix = False
        
        print("bag {} done".format(bag))
    print("All bags done")
