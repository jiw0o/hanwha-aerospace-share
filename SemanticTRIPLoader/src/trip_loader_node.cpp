/*
 * Maintainer: Minho Oh (minho.oh@kaist.ac.kr) @ Urban Robotics Lab, KAIST
 * This file is used to test the online TRIP algorithm on on-board system
 * Input: sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Imu, nav_msgs::msg::Odometry
 * Output: grid_map_msgs::GridMap
 *
 */

#define PCL_NO_PRECOMPILE
#include "trip_ros1/trip_loader_node.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros1_loader_node", ros::init_options::NoSigintHandler);
    std::cout << "\033[34;1m"
              << "TRIP Loader Node started"
              << "\033[32;0m" << std::endl;
    TRIP_LOADER_ROS1 trip_loader_ros1;
    ros::spin();
    trip_loader_ros1.~TRIP_LOADER_ROS1();
    return 0;
};
