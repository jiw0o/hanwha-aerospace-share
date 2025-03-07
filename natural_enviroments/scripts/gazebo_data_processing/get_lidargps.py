import rosbag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import NavSatFix
import numpy as np
import os

# Set your directories
root_direc = 'gazebo_kitti'
day_direc ='2024_08_12'
bag_list = ['lakepark1', 'lakepark2', 'lakepark3', 'lakepark4']

lidar_direc = 'velodyne_points/data'
lidar_topic_name = '/os0_cloud_node/points'

gps_direc = 'oxts/data_gps'
gps_topic_name = '/navsat/fix'

def get_index_offset(bag, lidar_topic_name, gps_topic_name):
    input_bag = os.path.join(root_direc, 'bag', day_direc, bag + '.bag')
    lidar_index = 0
    gps_index = 0

    with rosbag.Bag(input_bag, 'r') as input_bag:
        for _, _, t in input_bag.read_messages(topics=[lidar_topic_name]):
            lidar_index += 1
        for _, _, t in input_bag.read_messages(topics=[gps_topic_name]):
            gps_index += 1

    # assuming lidar messages are always more than gps messages
    index_offset = lidar_index - gps_index
    print("{} index offset for {}".format(index_offset, bag))
    return index_offset

def get_lidar(bag, lidar_direc, topic_name, index_offset):
    input_bag = os.path.join(root_direc, 'bag', day_direc, bag + '.bag')
    output_dir = os.path.join(root_direc, 'raw_data', day_direc, bag, lidar_direc)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    index = min(0 ,-index_offset)

    with rosbag.Bag(input_bag, 'r') as input_bag:
        for topic, msg, t in input_bag.read_messages(topics=[topic_name]):
            if index >= 0:
                point_cloud = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                all_points = np.array(list(point_cloud), dtype=np.float32)

                output_file = os.path.join(output_dir, '{:010d}.bin'.format(index))
                all_points.tofile(output_file)
                print("{} {:03d} LiDAR saved".format(bag, index))
            index += 1
            
        if index == 0:
            print(f"No LiDAR messages found in the specified topic: {topic_name}")
    print(f"LiDAR data has been written to {output_dir}")

def get_gps(bag, gps_direc, topic_name, index_offset):
    input_bag = os.path.join(root_direc, 'bag', day_direc, bag + '.bag')
    output_dir = os.path.join(root_direc, 'raw_data', day_direc, bag, gps_direc)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    index = min(0, index_offset)

    with rosbag.Bag(input_bag, 'r') as input_bag:
        for topic, msg, t in input_bag.read_messages(topics=[topic_name]):
            if index >= 0:
                lat = msg.latitude
                long = msg.longitude
                alt = msg.altitude

                output_file = os.path.join(output_dir, "{:010d}.txt".format(index))
                with open(output_file, 'w') as out_file:
                    out_file.write(f"{lat} {long} {alt}\n")
                print("{} {:03d} GPS saved".format(bag, index))
            index += 1
        
        if index == 0:
            print(f"No NavSatFix messages found in the specified topic: {topic_name}")
    print(f"GPS data has been written to {output_dir}")

if __name__ == '__main__':
    for bag in bag_list:
        index_offset = get_index_offset(bag, lidar_topic_name, gps_topic_name)
        get_lidar(bag, lidar_direc, lidar_topic_name, index_offset)        
        get_gps(bag, gps_direc, gps_topic_name, index_offset)