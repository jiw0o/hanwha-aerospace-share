#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#define PCL_NO_PRECOMPILE
#pragma once

#include <geometry_msgs/Pose.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <algorithm>
#include <boost/optional.hpp>
#include <chrono>
#include <cstdlib>
#include <experimental/filesystem>
#include <forward_list>
#include <fstream>
#include <iostream>
#include <list>
#include <mutex>
#include <numeric>
#include <random>
#include <sstream>
#include <thread>
#include <vector>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "trip_loader.hpp"

using ROS1_PTCLOUD = sensor_msgs::PointCloud2;
using ROS1_GRIDMAP = grid_map_msgs::GridMap;
using ROS1_ODOM = nav_msgs::Odometry;
using ROS1_HEADER = std_msgs::Header;
using ROS1_TF = tf::StampedTransform;

namespace utils_ros1 {

void getAlignMatfromStampedPose(trip_types::StampedPose& _pose_in, 
                                Eigen::Matrix4f& _global2aligned_transform, 
                                Eigen::Matrix4f& _local_gravity_align_transform) {
    double roll, pitch, yaw;
    Eigen::Quaternionf q = _pose_in.quat;
    Eigen::Matrix3f rot_mat = q.toRotationMatrix();
    roll = std::atan2(rot_mat(2, 1), rot_mat(2, 2));
    pitch = -std::asin(rot_mat(2, 0));
    yaw = std::atan2(rot_mat(1, 0), rot_mat(0, 0));

    _global2aligned_transform = pcl::getTransformation(_pose_in.pos[0], _pose_in.pos[1], _pose_in.pos[2], 0, 0, yaw).matrix();  // tf for align to body frame
    _local_gravity_align_transform = pcl::getTransformation(0, 0, 0, roll, pitch, 0).matrix();                                  // tf for align to body frame
    return;
};  // getAlignTFfromPose

void convertTFEigen2StampedPose(const Eigen::Matrix4f& _transform_eigen_in, 
                                trip_types::StampedPose& _pose_out) {
    _pose_out.pos[0] = _transform_eigen_in(0, 3);
    _pose_out.pos[1] = _transform_eigen_in(1, 3);
    _pose_out.pos[2] = _transform_eigen_in(2, 3);
    Eigen::Quaternionf q(_transform_eigen_in.block<3, 3>(0, 0));
    _pose_out.quat = q;
    return;
};

void convertOdom2Transform(const trip_types::StampedPose& _pose_in, tf::Transform& _transform_out) {
    // _transform_out.setOrigin(tf::Vector3(_pose_in.pose.pose.position.x, _pose_in.pose.pose.position.y, _pose_in.pose.pose.position.z));
    // _transform_out.setRotation(tf::Quaternion(_pose_in.pose.pose.orientation.x, _pose_in.pose.pose.orientation.y, _pose_in.pose.pose.orientation.z, _pose_in.pose.pose.orientation.w));
    
    _transform_out.setOrigin(tf::Vector3(_pose_in.pos[0], _pose_in.pos[1], _pose_in.pos[2]));
    _transform_out.setRotation(tf::Quaternion(_pose_in.quat.x(), _pose_in.quat.y(), _pose_in.quat.z(), _pose_in.quat.w()));
    
    return;
};

void initTRIPGridmap(grid_map::GridMap &grid_map_out, const std::string &grid_map_frame_id,
                     const float &map_width, const float &map_length, const float &resolution) {
    grid_map_out.setFrameId(grid_map_frame_id);
    grid_map_out.setGeometry(grid_map::Length(map_width, map_length), resolution);

    grid_map::Matrix elevation      (grid_map_out.getSize()(0), grid_map_out.getSize()(1)); elevation.setConstant(NAN);
    grid_map_out.add("elevation", elevation);
    // Field initialize by macro
    #define X(field)    grid_map::Matrix field(grid_map_out.getSize()(0), grid_map_out.getSize()(1)); \
                        field.setConstant(NAN); \
                        grid_map_out.add(#field, field);
    TRIP_FIELDS
    #undef X

    // clear all fields as NAN
    grid_map_out.clearAll();
    return;
};

void convertTRIPCloud2Gridmap(  const std::string &grid_map_frame_id, 
                                const pcl::PointCloud<trip_types::PointTRIP> &trip_cloud_in,
                                grid_map::GridMap &grid_map_out, float pose_x, float pose_y) {
    grid_map_out.clearAll();
    grid_map_out.setFrameId(grid_map_frame_id);
    grid_map_out.setPosition(grid_map::Position(pose_x, pose_y));

    for (const auto& pt : trip_cloud_in.points) {
        grid_map::Position pt2d(pt.x, pt.y);
        if (!grid_map_out.isInside(pt2d)) {
            continue;
        }

        grid_map::Index idx;
        grid_map_out.getIndex(pt2d, idx);

        if (std::isnan(grid_map_out.at("elevation", idx))) {
            // grid_map_out.at("elevation", idx) = pt.z;

            grid_map_out.at("elevation", idx) = pt.z;
            #define X(field) grid_map_out.at(#field, idx) = pt.field;
            TRIP_FIELDS
            #undef X
        } else {
            if (grid_map_out.at("elevation", idx) < pt.z) {
                continue;
            } else {
                // grid_map_out.at("elevation", idx) = pt.z;
                grid_map_out.at("elevation", idx) = pt.z;
                #define X(field) grid_map_out.at(#field, idx) = pt.field;
                TRIP_FIELDS
                #undef X
            }
            continue;
        }
    }

    grid_map::GridMap grid_map_tmp = grid_map_out;
    int x_size = grid_map_out.getSize()(0);
    int y_size = grid_map_out.getSize()(1);
    for (int x_i = 0 ; x_i < x_size; x_i++ ){
    for (int y_i = 0; y_i < y_size; y_i++) {
        grid_map::Index idx(x_i, y_i);
        if (!std::isnan(grid_map_tmp.at("elevation", idx))) {
            continue;
        }

        // For the empty cell, just copy the value from the neighbor cell
        float max_elevation = -1000;
        for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <=1; dy++) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            int x_n = x_i + dx;
            int y_n = y_i + dy;
            if (x_n < 0 || x_n >= x_size || y_n < 0 || y_n >= y_size) {
                continue;
            }
            grid_map::Index idx_n(x_n, y_n);
            if (!std::isnan(grid_map_tmp.at("elevation", idx_n))
                && grid_map_tmp.at("elevation", idx_n) > max_elevation) {

                grid_map_out.at("elevation", idx) = grid_map_tmp.at("elevation", idx_n);
                #define X(field) grid_map_out.at(#field, idx) = grid_map_tmp.at(#field, idx_n);
                TRIP_FIELDS
                #undef X
            }
        }
        }
    }
    }
    return;
};

template <typename PointT, typename ColorPointT>
void convertToColoredMap(const pcl::PointCloud<PointT>& cloud_in, 
                        pcl::PointCloud<ColorPointT>& cloud_out,
                        std::map<int, cv::Vec3b> &color_map){
    if (cloud_in.points.empty()) {
        return;
    }
    cloud_out.clear();
    cloud_out.reserve(cloud_in.size());
    for (auto pt : cloud_in.points) {
        
        // copy the point for each fields by macro (FIELDS of PointT)
        ColorPointT pt_out;
        
        pt_out.x = pt.x;
        pt_out.y = pt.y;
        pt_out.z = pt.z;
        pt_out.label = pt.intensity;
        if (pt.intensity < 0) {
            pt_out.r = 255;
            pt_out.g = 255;
            pt_out.b = 255;
        } else {
            pt_out.r = (uint8_t) color_map[pt.intensity][0];
            pt_out.g = (uint8_t) color_map[pt.intensity][1];
            pt_out.b = (uint8_t) color_map[pt.intensity][2];
        }
        // std::cout << "color: " << (int) pt_out.r << ", " << (int) pt_out.g << ", " << (int) pt_out.b << std::endl;
        cloud_out.push_back(pt_out);
    }
    return;
};

void publishTRIPGridmap (const std_msgs::Header & f_header_in, const grid_map::GridMap &grid_map_out, const ros::Publisher &pub) {
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(grid_map_out, grid_map_msg);
    grid_map_msg.info.header = f_header_in;
    pub.publish(grid_map_msg);
};

template <typename T>
void publishCloud(const ROS1_HEADER& header, boost::shared_ptr<pcl::PointCloud<T>>& cloud, ros::Publisher& pub) {
    ROS1_PTCLOUD cloud_msg;
    cloud->width = cloud->size();
    cloud->height = 1;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header = header;
    pub.publish(cloud_msg);
    return;
};

void publishOdometry(const ros::Time& stamp, 
                    const std::string src_frame_id, 
                    const std::string tgt_frame_id, 
                    const tf::Transform& transform, 
                    ros::Publisher& pubOdom, 
                    tf::TransformBroadcaster& tf_br) {
    ROS1_ODOM odom_topic;
    odom_topic.header.frame_id = src_frame_id;
    odom_topic.child_frame_id = tgt_frame_id;
    odom_topic.header.stamp = stamp;
    odom_topic.pose.pose.position.x = transform.getOrigin().x();
    odom_topic.pose.pose.position.y = transform.getOrigin().y();
    odom_topic.pose.pose.position.z = transform.getOrigin().z();
    odom_topic.pose.pose.orientation.x = transform.getRotation().x();
    odom_topic.pose.pose.orientation.y = transform.getRotation().y();
    odom_topic.pose.pose.orientation.z = transform.getRotation().z();
    odom_topic.pose.pose.orientation.w = transform.getRotation().w();

    pubOdom.publish(odom_topic);

    ROS1_TF tf_odom;
    tf_odom.stamp_ = stamp;
    tf_odom.frame_id_ = src_frame_id;
    tf_odom.child_frame_id_ = tgt_frame_id;
    tf_odom.setOrigin(tf::Vector3(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()));
    tf_odom.setRotation(tf::Quaternion(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w()));
    tf_br.sendTransform(tf_odom);
    return;
};

double get_time_sec(const ros::Time& time) { return time.toSec(); };

};  // namespace utils_ros1

#endif  // ROS_UTILS_HP-0]iu
