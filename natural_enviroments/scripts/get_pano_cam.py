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
# day_direc ='2024_07_02'
day_direc ='2024_08_08'
bag_list = ['lakepark1', 'lakepark2', 'lakepark3']
rpy_direc = 'oxts/data_rpy'
image_direc = 'image_02/pano'

# Set what to save
# is_rpy = True
is_grdcam = True

im_left_check=False
im_right_check=False
pose_check=False

# img_msg_right=CompressedImage()
# img_msg_left=CompressedImage()
img_msg_right=Image()
img_msg_left=Image()

def left_img_cb(msg):
    global img_msg_left
    img_msg_left=msg
    im_left_check=True

def right_img_cb(msg): 
    global img_msg_right
    img_msg_right=msg
    # im_right_check=True

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
    # euler = tf.transformations.euler_from_quaternion(quaternion)
    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
    # return euler  # roll, pitch, yaw
    return yaw

def get_image(image, image_direc, index, orientation):
    image_cv = bridge.imgmsg_to_cv2(image, "bgr8")
    yaw = quaternion_to_euler(orientation)
    # shift image to north aligned
    print("yaw: ", yaw)
    print("width:", image_cv.shape[1])
    shift = int(yaw/(2*math.pi)*image_cv.shape[1])
    print("shift:", shift)
    image_north = np.roll(image_cv, shift, axis=1)
    # shift
    output_dir = os.path.join(root_direc, 'raw_data', day_direc, bag, image_direc)
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

        rospy.init_node('im_listener')
        # rospy.Subscriber("/husky/camera/left/image_raw/compressed",CompressedImage,left_img_cb)
        # rospy.Subscriber("/husky/camera/right/image_raw/compressed",CompressedImage,right_img_cb)
        rospy.Subscriber("/husky/camera/pano/image_raw",Image,left_img_cb)
        # rospy.Subscriber("/husky/camera/right/image_raw",Image,right_img_cb)
        bridge = CvBridge()
        t_prev=rospy.Time()

        index = 0
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
                msg_move.model_name='pano_camera'
                msg_move.pose=msg.pose[len(msg.name)-1]
                msg_move.pose.position.x=msg.pose[len(msg.name)-1].position.x+0.0812
                msg_move.pose.position.z=msg.pose[len(msg.name)-1].position.z+0.245+0.52
                orientation = msg_move.pose.orientation

                # # for satelite camera
                # msg_move.pose.orientation.x = 0.0
                # msg_move.pose.orientation.y = 0.7071068
                # msg_move.pose.orientation.z = 0.0
                # msg_move.pose.orientation.w = 0.7071068
                # msg_move.pose.position.z=40
                
                rospy.wait_for_service('/gazebo/set_model_state')
                try:
                    set_state = rospy.ServiceProxy(
                        '/gazebo/set_model_state', SetModelState)
                    resp = set_state(msg_move)

                except rospy.ServiceException as e:
                    print ("Service call failed: %s" % e)

                rospy.sleep(0.01)
                print(resp)

                im_left_check=False
                im_right_check=False
                pose_check=False
                
                if is_navsat_fix:
                    # if is_rpy:
                    #     get_rpy(orientation, rpy_direc, index)
                    if is_grdcam:
                        get_image(img_msg_left, image_direc, index)
                    index += 1
                    is_navsat_fix = False
                
