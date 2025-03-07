#ifndef COMMON_UTILS_HPP
#define COMMON_UTILS_HPP

#pragma once

#include <math.h>
#include <omp.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <boost/optional.hpp>
#include <chrono>
#include <forward_list>
#include <iostream>
#include <list>
#include <mutex>
#include <numeric>
#include <random>
#include <thread>
#include <vector>

// #define TRIP_FIELDS \
//     X(max_elevation) \
//     X(min_elevation) \
//     X(verticality) \
//     X(steepness_risk) \
//     X(inclination_risk) \
//     X(collision_risk) \
//     X(normal_x) \
//     X(normal_y) \
//     X(normal_z) \
//     X(height_noise) \
//     X(normal_noise) \
//     X(confidence)

#define TRIP_FIELDS \
    X(normal_x) \
    X(normal_y) \
    X(normal_z) \
    X(intensity) \
    X(min_elevation) \
    X(verticality) \
    X(steepness_risk) \
    X(inclination_risk) \
    X(collision_risk) \
    X(height_noise) \
    X(normal_noise) \
    X(confidence)

namespace signal_handle {
void signal_callback_handler(int signum) {
    std::cout << "\033[34;1m"
              << "Caught signal " << signum << "\033[32;0m" << std::endl;
    // Terminate program
    exit(signum);
}
}  // namespace signal_handle

namespace trip_types {
    struct StampedPose {
        Eigen::Vector3f pos;
        Eigen::Quaternionf quat;
        double stamp;
        std::string frame_id;
        std::string child_frame_id;

        Eigen::Matrix4f getTransformMatrix() {
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block<3, 3>(0, 0) = quat.toRotationMatrix();
            T.block<3, 1>(0, 3) = pos;
            return T;
        };
    };

    struct PointTRIP {
        PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
        PCL_ADD_NORMAL4D;
        PCL_ADD_INTENSITY;
        // float max_elevation;  // difference of height in same cell: max_height - min_height
        float min_elevation;  // difference of height in same cell: max_height - min_height

        float verticality;        // verticality of the point 0.0 - 1.0
        float steepness_risk;  // risk cost from traversability 0.0 - 1.0
        float inclination_risk;   // risk cost from elevation diff 0.0 - 1.0
        float collision_risk;     // occupied probability of the point

        float height_noise;  // if the point is observed -> 0, if the point is predicted -> over then 0
        float normal_noise;

        float confidence;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;

    struct ColorPointTRIP {
        PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
        PCL_ADD_INTENSITY;
        PCL_ADD_RGB;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;

    template <typename PointT>
    void transformTerrainMap(const pcl::PointCloud<PointT>& cloud_in, 
                            pcl::PointCloud<PointT>& cloud_out, 
                            Eigen::Matrix4f& transform) {
        cloud_out = cloud_in;
        Eigen::Vector4f pt_in_eig, pt_in_max_eig, pt_in_min_eig;
        Eigen::Vector4f pt_out_eig, pt_out_max_eig, pt_out_min_eig;
        Eigen::Vector3f pt_in_normal, pt_out_normal;
        
        for (int i = 0; i < (int) cloud_out.size(); i++) {
            PointT pt = cloud_in.points[i];
            pt_in_eig << pt.x, pt.y, pt.z, 1;
            // pt_in_max_eig << pt.x, pt.y, pt.max_elevation, 1;
            pt_in_min_eig << pt.x, pt.y, pt.min_elevation, 1;
            
            pt_out_eig = transform * pt_in_eig;
            // pt_out_max_eig = transform * pt_in_max_eig;
            pt_out_min_eig = transform * pt_in_min_eig;
            
            pt_in_normal << pt.normal_x, pt.normal_y, pt.normal_z;
            pt_out_normal = transform.block<3, 3>(0, 0) * pt_in_normal;
            
            cloud_out.points[i].x = pt_out_eig[0];
            cloud_out.points[i].y = pt_out_eig[1];
            cloud_out.points[i].z = pt_out_eig[2];
            cloud_out.points[i].normal_x = pt_out_normal[0];
            cloud_out.points[i].normal_y = pt_out_normal[1];
            cloud_out.points[i].normal_z = pt_out_normal[2];
            // cloud_out.points[i].max_elevation = pt_out_max_eig[2];
            cloud_out.points[i].min_elevation = pt_out_min_eig[2];

        }
        return;
    };

};  // namespace trip_types

POINT_CLOUD_REGISTER_POINT_STRUCT(trip_types::PointTRIP,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, normal_x, normal_x)
                                    (float, normal_y, normal_y)
                                    (float, normal_z, normal_z)
                                    (float, intensity, intensity)
                                    (float, min_elevation, min_elevation)
                                    (float, verticality, verticality)
                                    (float, steepness_risk, steepness_risk)
                                    (float, inclination_risk, inclination_risk)
                                    (float, collision_risk, collision_risk)
                                    (float, height_noise, height_noise)
                                    (float, normal_noise, normal_noise)
                                    (float, confidence, confidence))

POINT_CLOUD_REGISTER_POINT_STRUCT(trip_types::ColorPointTRIP,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (std::uint8_t , r, r)
                                    (std::uint8_t , g, g)
                                    (std::uint8_t , b, b))

using PtRaw = pcl::PointXYZI;
using PtTRIP = trip_types::PointTRIP;
// using PtColorTRIP = trip_types::ColorPointTRIP;
using PtColorTRIP = pcl::PointXYZRGBL;

#endif  // UTILS_COMMON_HPP