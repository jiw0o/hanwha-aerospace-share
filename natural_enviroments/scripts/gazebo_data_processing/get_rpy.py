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
im_left_check=False
im_rigth_check=False
pose_check=False
# img_msg_right=CompressedImage()
# img_msg_left=CompressedImage()
# img_msg_right=Image()
# img_msg_left=Image()
def left_img_cb(msg):
     global img_msg_left
     img_msg_left=msg
     im_left_check=True
def right_img_cb(msg): 
     global img_msg_right
     img_msg_right=msg
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

def extract_pose_data_to_files(orientation, output_dir, index):
    roll, pitch, yaw = quaternion_to_euler(orientation)

    output_file = os.path.join(output_dir, "{:010d}.txt".format(index))
    with open(output_file, 'w') as out_file:
        # out_file.write(f"{roll} {pitch} {yaw}\n")
        out_file.write("{:.6f} {:.6f} {:.6f}\n".format(roll, pitch, yaw))

if __name__ == '__main__':
  global img_msg_left
  global img_msg_right
  global im_rigth_check
  global pose_check

  bridge = CvBridge()
  inputfile = ''
  outputfile = ''
  
  try:
      opts, args = getopt.getopt(sys.argv[1:],"hi:o:",["ifile=","ofile="])
  except getopt.GetoptError:
      print 'test.py -i <inputfile> -o <outputfile>'
      sys.exit(2)
  for opt, arg in opts:
      if opt == '-h':
         print 'test.py -i <inputfile> -o <outputfile>'
         sys.exit()
      elif opt in ("-i", "--ifile"):
         inputfile = arg
      elif opt in ("-o", "--ofile"):
         outputfile = arg

  inputbag = 'lakepark3'
  output_dir = os.path.join('gazebo_data', inputbag, 'oxts', 'data_rpy')
  inputbag = inputbag + '.bag'
  if not os.path.exists(output_dir):
      os.makedirs(output_dir)

  rospy.init_node('im_listener')
#   rospy.Subscriber("/husky/camera/left/image_raw/compressed",CompressedImage,left_img_cb)
#   rospy.Subscriber("/husky/camera/right/image_raw/compressed",CompressedImage,right_img_cb)
#   rospy.Subscriber("/husky/camera/left/image_raw",Image,left_img_cb)
#   rospy.Subscriber("/husky/camera/right/image_raw",Image,right_img_cb)
  t_prev=rospy.Time()
  index = 0
  is_navsat_fix = False
  msg_move = None
  with rosbag.Bag(outputfile, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(inputfile).read_messages():
       #if topic != "/stereo/camera/right/image_raw_tagged":
           #outbag.write(topic, msg, t)
    #    print("testing")
       outbag.write(topic, msg, t)

       if topic== "/navsat/fix":
           is_navsat_fix = True

       if topic == "/gazebo/model_states" and is_navsat_fix==True:

            t_prev=t

            msg_move=ModelState()
            msg_move.model_name='stereo_camera'
            #Se usa el cambio baselink->cameralink para posicionarla correctamente
            msg_move.pose=msg.pose[len(msg.name)-1]
            # msg_move.pose.position.x=msg.pose[len(msg.name)-1].position.x+0.0812
            # msg_move.pose.position.z=msg.pose[len(msg.name)-1].position.z+0.245+0.52
            orientation = msg_move.pose.orientation
            
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
            im_rigth_check=False
            pose_check=False

            # if is_navsat_fix==True:
            print(f"{index} pose data extracted") 
            extract_pose_data_to_files(orientation, output_dir, index)
            # outbag.write('stereo/camera/left/image_raw', img_msg_left, t)
            # outbag.write('stereo/camera/right/image_raw', img_msg_right, t)
            index += 1
            is_navsat_fix = False
              
