#!/usr/bin/env python  
import rosbag
import tf
from std_msgs.msg import Float32,Header
from sensor_msgs.msg import CameraInfo, Image,CompressedImage
from gazebo_msgs.srv import SpawnModel, DeleteModel,SetModelState
from gazebo_msgs.msg import ModelState,ModelStates
from cv_bridge import CvBridge
import cv2
import getopt
import matplotlib.pyplot as plt
import os

def quaternion_to_euler(orientation):
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler  # roll, pitch, yaw

def extract_pose_data_to_files(pose, output_dir, index):
    roll, pitch, yaw = quaternion_to_euler(pose.orientation)

    # Write the data in the specified format
    output_file = os.path.join(output_dir, f"{index:010d}.txt")
    with open(output_file, 'w') as out_file:
        out_file.write(f"{roll} {pitch} {yaw}\n")

if __name__ == '__main__':
    inputbag = 'lakepark1'
    output_dir = os.path.join('gazebo_data', inputbag, 'oxts', 'data')
    inputbag = inputbag + '.bag'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    is_navsat_fix = False
    
    index = 0
    with rosbag.Bag(inputbag, 'r') as bag:
        for topic, msg, t in rosbag.Bag(inputbag).read_messages():

            if topic== "/navsat/fix":
                is_navsat_fix = True

            if topic == "/gazebo/model_states" and is_navsat_fix:
                msg_move = ModelStates()
                msg_move.pose=msg.pose[len(msg.name)-1]
                msg_move.pose
                extract_pose_data_to_files(msg, output_dir, index)
                print("TEST")
                index += 1
                is_navsat_fix = False
              
