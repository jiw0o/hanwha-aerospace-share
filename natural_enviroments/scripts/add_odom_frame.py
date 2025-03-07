#!/usr/bin/env python  
import rosbag
import tf
import sys
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Pose
from tf2_msgs.msg import TFMessage
import getopt


def pose_to_matrix(pose):
    """Convert a ROS pose (position and orientation) to a 4x4 transformation matrix."""
    trans = tf.transformations.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
    rot = tf.transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    return np.dot(trans, rot)

def matrix_to_transform_pose(matrix):
    """Convert a 4x4 transformation matrix back to a ROS pose."""
    translation = tf.transformations.translation_from_matrix(matrix)
    quaternion = tf.transformations.quaternion_from_matrix(matrix)
    
    transform = TransformStamped()
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    
    pose = Pose()
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return transform, pose

def inverse_transform(matrix):
    """Invert a 4x4 transformation matrix."""
    return tf.transformations.inverse_matrix(matrix)

def msg_header(msg, seq, stamp, frame_id, child_frame_id):
    msg.header.seq = seq
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    return msg

if __name__ == "__main__":
    # Initialize variables for input and output bag files
    inputfile = ''
    outputfile = ''
    
    # Parse command line options
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hi:o:", ["ifile=", "ofile="])
    except getopt.GetoptError:
        print('Usage: script.py -i <inputfile> -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('Usage: script.py -i <inputfile> -o <outputfile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg

    # Check if both input and output files are specified
    if not inputfile or not outputfile:
        print('Both input and output files must be specified.')
        print('Usage: script.py -i <inputfile> -o <outputfile>')
        sys.exit(2)
    
    print("Input file: {}".format(inputfile))
    print("Output file: {}".format(outputfile))
    
    # Now, you can use `inputfile` and `outputfile` in the logic of your script
    # For example, the earlier logic for reading and writing bag files:
    
    map_to_odom_matrix = None
    map_to_odom_initialized = False

    with rosbag.Bag(outputfile, 'w') as outbag:
        with rosbag.Bag(inputfile, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                # Your logic for handling messages goes here
                # For example:
                outbag.write(topic, msg, t)

                # Processing based on /odometry_gt, /tf_static, /tf, etc.
                if topic == "/odometry_gt":
                    seq = msg.header.seq
                    stamp = msg.header.stamp
                    map_to_base_link_matrix = pose_to_matrix(msg.pose.pose)

                    if not map_to_odom_initialized:
                        map_to_odom_matrix = map_to_base_link_matrix
                        tf_static = TFMessage()
                        tf_map_to_odom, _ = matrix_to_transform_pose(map_to_odom_matrix)
                        tf_map_to_odom = msg_header(tf_map_to_odom, seq, stamp, "map", "odom")
                        tf_static.transforms.append(tf_map_to_odom)
                        outbag.write('/tf_static', tf_static, t)

                        # odom to base_link: identity
                        odom_to_base_link_matrix = np.eye(4)
                        tf_msg = TFMessage()
                        tf_odom_to_base_link, _ = matrix_to_transform_pose(odom_to_base_link_matrix)
                        tf_odom_to_base_link = msg_header(tf_odom_to_base_link, seq, stamp, "odom", "base_link")
                        tf_msg.transforms.append(tf_odom_to_base_link)
                        outbag.write('/tf', tf_msg, t)

                        map_to_odom_initialized = True

                    else:
                        odom_to_base_link_matrix = np.dot(inverse_transform(map_to_odom_matrix), map_to_base_link_matrix)
                        tf_odom_to_base_link, pose_odom_to_base_link = matrix_to_transform_pose(odom_to_base_link_matrix)

                        tf_msg = TFMessage()
                        tf_odom_to_base_link = msg_header(tf_odom_to_base_link, seq, stamp, "odom", "base_link")
                        tf_msg.transforms.append(tf_odom_to_base_link)
                        outbag.write('/tf', tf_msg, t)

                        odom_msg = Odometry()
                        odom_msg = msg_header(odom_msg, seq, stamp, "odom", "base_link")
                        odom_msg.pose.pose = pose_odom_to_base_link
                        outbag.write('/odometry', odom_msg, t)
                    
                    print("Seq: {} processed".format(seq))