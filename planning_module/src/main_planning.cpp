#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <limits>
#include <valarray>
#include <tuple>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GetGridMapInfo.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// OMPL
#include <ompl/config.h>
#include <ompl/util/RandomNumbers.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/OptimizationObjective.h>
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include <ompl/base/goals/GoalSampleableRegion.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/TSRRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>

#include <ompl/control/ODESolver.h>
#include <ompl/control/Control.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>

#include <planning_module/PathData.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Point {
    double x;
    double y;
};

std::string global_frame_id = "/map";
std::string robot_frame_id = "/base_link";
std::string odom_topic_name = "/odometry_gt";


//  ██████╗ ██████╗ ███████╗████████╗    ███╗   ███╗ █████╗ ██████╗     
// ██╔════╝██╔═══██╗██╔════╝╚══██╔══╝    ████╗ ████║██╔══██╗██╔══██╗    
// ██║     ██║   ██║███████╗   ██║       ██╔████╔██║███████║██████╔╝    
// ██║     ██║   ██║╚════██║   ██║       ██║╚██╔╝██║██╔══██║██╔═══╝     
// ╚██████╗╚██████╔╝███████║   ██║       ██║ ╚═╝ ██║██║  ██║██║         
//  ╚═════╝ ╚═════╝ ╚══════╝   ╚═╝       ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝         

class GridMapHandler {
public:
    grid_map::GridMap gridMap;
    bool map_ready_flag = false;

    GridMapHandler() : nh_private_("~") {
        // 퍼블리셔 초기화
        grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/GridMap_planning", 1, true);

        // ROS 파라미터 로드
        loadParameters();

    }

    /**
     * @brief GridMap 메시지를 콜백하여 처리합니다.
     * @param msg GridMap 메시지 포인터
     */
    void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg) {
        if (!map_ready_flag) {
            auto start_time = std::chrono::high_resolution_clock::now();

            loadParameters();

            gridMap.clearAll();
            grid_map::GridMapRosConverter::fromMessage(*msg, gridMap);

            computeMapCorners();
            initializeLayers();

            // 기하적 위험도 맵 생성 및 처리
            computeGeometricCostMap();
            processCostMap("geometric_cost_map", "geometric_cost_map_filtered", 
                          geometric_cost_threshold_, robot_radius_, geometric_dilation_alpha_, geometric_gaussian_sigma_);

            // 시맨틱 위험도 맵 생성 및 처리
            computeSemanticCostMap();
            processCostMap("semantic_cost_map", "semantic_cost_map_filtered", 
                          semantic_cost_threshold_, robot_radius_, semantic_dilation_alpha_, semantic_gaussian_sigma_);

            // 최종 비용 맵 생성
            generateFinalCostMap();

            // 최종 맵 퍼블리시
            publishPlanningMap();

            map_ready_flag = true;
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> processing_time = end_time - start_time;
            ROS_WARN(">>>>>>> Received [GridMap]:");
            ROS_INFO("[Map] Center (x, y): (%f, %f)", map_center_x, map_center_y);
            ROS_INFO("[Map] Top Left (x, y): (%f, %f)", map_topleft_corner_x, map_topleft_corner_y);
            ROS_INFO("[Map] Bottom Right (x, y): (%f, %f)", map_bottomright_corner_x, map_bottomright_corner_y);
            ROS_INFO("[Map] GridMap processing time: %f seconds", processing_time.count());
            ROS_WARN("-----------------------------------------------------------------------");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    double map_center_x = 0.0, map_center_y = 0.0;
    double map_topleft_corner_x = 0.0, map_topleft_corner_y = 0.0;
    double map_bottomright_corner_x = 0.0, map_bottomright_corner_y = 0.0;

    ros::Publisher grid_map_pub_;

    // 로봇 반경 및 기타 하이퍼파라미터
    double robot_radius_;

    double geometric_dilation_alpha_;
    double geometric_gaussian_sigma_;
    double geometric_cost_threshold_;
    double geometric_cost_amplification_;

    double semantic_dilation_alpha_;
    double semantic_gaussian_sigma_;
    double semantic_cost_threshold_;
    double semantic_cost_amplification_;

    bool is_fusion_using_max_;

    int aggressive_level_; // CVaR aggressive value

    bool is_geo_treat_unknown_as_obstacle_;
    bool is_sem_treat_unknown_as_obstacle_;

    /**
     * @brief ROS 파라미터를 로드합니다.
     */
    void loadParameters() {
        // 예시: 파라미터 이름은 필요에 따라 조정 가능
        nh_private_.param("robot_radius", robot_radius_, 0.6);
        nh_private_.param("geometric_dilation_alpha", geometric_dilation_alpha_, 1.4);
        nh_private_.param("geometric_gaussian_sigma", geometric_gaussian_sigma_, 2.0);
        nh_private_.param("semantic_dilation_alpha", semantic_dilation_alpha_, 1.4);
        nh_private_.param("semantic_gaussian_sigma", semantic_gaussian_sigma_, 2.0);

        nh_private_.param("geometric_cost_threshold", geometric_cost_threshold_, 0.7);
        nh_private_.param("semantic_cost_threshold", semantic_cost_threshold_, 0.7);

        nh_private_.param("geometric_cost_amplification", geometric_cost_amplification_, 1.2);
        nh_private_.param("semantic_cost_amplification", semantic_cost_amplification_, 1.2);

        nh_private_.param("is_fusion_using_max", is_fusion_using_max_, true);

        nh_private_.param("aggressive_level", aggressive_level_, 1);

        nh_private_.param("is_geo_treat_unknown_as_obstacle", is_geo_treat_unknown_as_obstacle_, false);
        nh_private_.param("is_sem_treat_unknown_as_obstacle", is_sem_treat_unknown_as_obstacle_, false);
    }

    void processCostMap(const std::string& input_layer, const std::string& output_layer,
                       double threshold, double radius, double dilation_alpha, double gaussian_sigma) {
        // 1. 입력 레이어를 OpenCV 이미지로 변환
        cv::Mat cost_image;
        grid_map::GridMapCvConverter::toImage<float, 1>(
            gridMap, input_layer, CV_32FC1, 0.0, 1.0, cost_image);

        // 2. 비용이 threshold 이상인 부분을 이진화 ----------------------------------------------
        cv::Mat binary_image;
        double threshold_value = threshold; // 필요에 따라 조정 가능
        cv::threshold(cost_image, binary_image, threshold_value, 255, cv::THRESH_BINARY);
        binary_image.convertTo(binary_image, CV_8UC1);

        // 2.5. 소금-후추 노이즈 제거 (Median Blur)
        cv::Mat denoised_image;
        cv::medianBlur(binary_image, denoised_image, 3);

        // 3. 홀 필링
        cv::Mat hole_filled;
        fillHoles(denoised_image, hole_filled, threshold);

        // 4. 다일레이션 
        int dilation_radius = static_cast<int>(std::round(radius * dilation_alpha / gridMap.getResolution()));
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(2 * dilation_radius + 1, 2 * dilation_radius + 1),
                                                    cv::Point(dilation_radius, dilation_radius));
        cv::Mat dilated_image;
        cv::dilate(hole_filled, dilated_image, element);
        dilated_image.convertTo(dilated_image, CV_32FC1, 1.0 / 255.0);

        // 5. 원본 비용 이미지에 다일레이션된 영역을 반영 (비용을 1로 설정) ---------------------------
        cv::Mat dilation_mask = (dilated_image == 1.0f);
        cost_image.setTo(1.0f, dilation_mask);

        // 6. 가우시안 필터 적용
        int kernel_size = static_cast<int>(std::round(radius * gaussian_sigma / gridMap.getResolution())) * 2 + 1;
        if (kernel_size % 2 == 0) {
            kernel_size += 1;
        }
        cv::Mat gaussian_image;
        cv::GaussianBlur(cost_image, gaussian_image, cv::Size(kernel_size, kernel_size), 0.0);
        gaussian_image.setTo(1.0f, dilation_mask);

        // 7. 필터링된 이미지를 출력 레이어에 추가
        grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(
            gaussian_image, output_layer, gridMap, 0.0, 1.0);
    }

    void fillHoles(const cv::Mat& binary_image, cv::Mat& filled_image, double threshold_value = 0.99) {
        cv::Mat inverted_binary;
        cv::bitwise_not(binary_image, inverted_binary);

        cv::Mat flood_filled = inverted_binary.clone();
        std::vector<cv::Point> flood_start_points = {};
        cv::Point center_point(int(binary_image.cols / 2), int(binary_image.rows / 2));
        if (inverted_binary.at<uchar>(center_point) == 255) {
            int connectivity = 4;
            int flood_fill_color = 255;
            // Flood Fill 여러 지점에서 수행
            cv::floodFill(flood_filled, cv::Mat(), center_point, flood_fill_color, 0, 0, connectivity);
        }
        
        // Flood Filled 이미지 반전
        cv::Mat flood_filled_inverted;
        cv::bitwise_not(flood_filled, flood_filled_inverted);
        
        // 홀 채우기
        cv::bitwise_or(binary_image, flood_filled_inverted, filled_image);
        
        int morph_iterations = 4;
        // 형태학적 폐쇄 적용 (옵션)
        if(morph_iterations > 0) {
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(filled_image, filled_image, cv::MORPH_CLOSE, element, cv::Point(-1, -1), morph_iterations);
        }
    }

    void generateFinalCostMap() {
        for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);

            double geo_cost = gridMap.at("geometric_cost_map_filtered", index);
            if (gridMap.at("geometric_cost_map", index) >= geometric_cost_threshold_) {
                geo_cost = 1.0;
            } else {
                geo_cost *= geometric_cost_amplification_;
            }

            double sem_cost = gridMap.at("semantic_cost_map_filtered", index);
            if (sem_cost >= semantic_cost_threshold_) {
                sem_cost = 1.0;
            } else {
                sem_cost *= semantic_cost_amplification_;
            }

            // 최종 비용 계산
            double updated_cost = 0.0;
            if (is_fusion_using_max_){
                updated_cost = std::max(geo_cost, sem_cost);
            } else {
                updated_cost = geo_cost + sem_cost;
            }

            if (updated_cost > 1.0) {
                updated_cost = 1.0;
            }

            grid_map::Position position;
            gridMap.getPosition(index, position);
            // 원점에서 반경 robot_radius_ 내의 cost 설정
            if (position.norm() <= robot_radius_) {
                updated_cost = 0.0;
            }

            gridMap.at("planning_cost_map_updated", index) = updated_cost;
        }
    }

    /**
     * @brief 지정된 레이어가 존재하지 않으면 추가합니다.
     * @param layer_name 추가할 레이어의 이름
     * @param default_value 레이어의 기본 값
     */
    void addLayerIfNotExist(const std::string& layer_name, float default_value) {
        if (!gridMap.exists(layer_name)) {
            gridMap.add(layer_name, default_value);
        }
    }

    /**
     * @brief 맵의 중심 및 코너 위치를 계산합니다.
     */
    void computeMapCorners() {
        const grid_map::Position map_center_position = gridMap.getPosition();
        map_center_x = map_center_position.x();
        map_center_y = map_center_position.y();
        const grid_map::Length mapLength = gridMap.getLength();
        double map_length_x = mapLength(0);
        double map_length_y = mapLength(1);
        map_topleft_corner_x = map_center_x + map_length_x / 2;
        map_topleft_corner_y = map_center_y + map_length_y / 2;
        map_bottomright_corner_x = map_center_x - map_length_x / 2;
        map_bottomright_corner_y = map_center_y - map_length_y / 2;
    }

    /**
     * @brief 필요한 레이어를 초기화합니다.
     */
    void initializeLayers() {
        // 기하적 비용 관련 레이어
        addLayerIfNotExist("_geometric_map_nan", 0.0);
        addLayerIfNotExist("geometric_cost_map", 0.0);
        addLayerIfNotExist("geometric_cost_map_filtered", 0.0);

        // 시맨틱 비용 관련 레이어
        addLayerIfNotExist("semantic_cost_map", 0.0);
        addLayerIfNotExist("semantic_cost_map_filtered", 0.0);
        addLayerIfNotExist("_semantic_map_0", 0.0);
        addLayerIfNotExist("_semantic_map_1", 0.0);
        addLayerIfNotExist("_semantic_map_2", 0.0);
        addLayerIfNotExist("_semantic_map_3", 0.0);
        addLayerIfNotExist("_semantic_map_4", 0.0);
        addLayerIfNotExist("_semantic_map_5", 0.0);
        addLayerIfNotExist("_semantic_map_6", 0.0);
        addLayerIfNotExist("_semantic_map_unlabeled", 0.0);
        addLayerIfNotExist("_semantic_map_nan", 0.0);

        // 최종 계획 비용 맵
        addLayerIfNotExist("planning_cost_map_updated", 0.0);
    }

    /**
     * @brief 기하적 위험도 맵을 계산합니다.
     */
    void computeGeometricCostMap() {
        for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            double inclination_risk = gridMap.at("inclination_risk", index);
            double collision_risk = gridMap.at("collision_risk", index);
            double steepness_risk = gridMap.at("steepness_risk", index);

            if (std::isnan(inclination_risk)|| std::isnan(collision_risk) || std::isnan(steepness_risk)) {
                gridMap.at("_geometric_map_nan", index) = 1.0;
                if (is_geo_treat_unknown_as_obstacle_) {
                    gridMap.at("geometric_cost_map", index) = 1.0;
                }else{
                    gridMap.at("geometric_cost_map", index) = 0.0;
                }
                continue;
            }

            double risk_cost = computeRiskCost(inclination_risk, collision_risk, steepness_risk);
            gridMap.at("geometric_cost_map", index) = risk_cost;
        }
    }

    /**
     * @brief CVaR 기반으로 위험 비용을 계산합니다.
     * @param incl 위험도 지표
     * @param coll 위험도 지표
     * @param steep 위험도 지표
     * @return 계산된 위험 비용
     */
    double computeRiskCost(double incl, double coll, double steep) {
        double risk_cost = 0.0;
        if (aggressive_level_ == 1) {
            risk_cost = std::max({incl, coll, steep});
        } else if (aggressive_level_ == 2) {
            std::vector<double> risks = {incl, coll, steep};
            std::sort(risks.rbegin(), risks.rend());
            risk_cost = (risks[0] + risks[1]) / 2.0;
        } else {
            risk_cost = (incl + coll + steep) / 3.0;
        }
        return risk_cost;
    }

    /**
     * @brief 시맨틱 위험도 맵을 계산합니다.
     */
    void computeSemanticCostMap() {
        std::set<int> labels = {0, 1, 2, 3, 4, 5, 6};
        for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            double label_double = gridMap.at("intensity", index);
            if (std::isnan(label_double)) {
                gridMap.at("_semantic_map_nan", index) = 1.0;
                if (is_sem_treat_unknown_as_obstacle_) {
                    gridMap.at("semantic_cost_map", index) = 1.0;
                }
                continue;
            } 

            int label_int = static_cast<int>(label_double);
            if (labels.find(label_int) != labels.end()) {
                std::string layer_name = "_semantic_map_" + std::to_string(label_int);
                gridMap.at(layer_name, index) = 1.0;
            } else {
                gridMap.at("_semantic_map_unlabeled", index) = 1.0;
            }

            // 시맨틱 비용 계산
            double semantic_cost = computeSemanticCost(label_int);
            gridMap.at("semantic_cost_map", index) = semantic_cost;
        }
    }

    /**
     * @brief 시맨틱 비용을 계산합니다.
     * @param label 시맨틱 레이블
     * @return 계산된 시맨틱 비용
     */
    double computeSemanticCost(int label) {
        double semantic_cost = 0.0;
        switch (label) {
            case 0: // void (장애물, 사람, 빌딩, 그 외) | 작은나무, 큰돌, 작은돌 (in Gazebo)
            case 2: // tree (나무) | 큰나무
            case 3: // water (물, 물웅덩이) | 연못
                semantic_cost = 1.0;
                break;
            case 1: // grass (잔디) | 잔디
            case 6: // rubble (자갈, 돌) | X
                semantic_cost = 0.3;
                break;
            case 4: // sky (하늘) | X
            case 5: // asphalt (인도) | 흙길
            default: // unlabeled (미분류) | X
                semantic_cost = 0.0;
                break; // No additional cost for sky and default case
        }
        return semantic_cost;
    }

    /**
     * @brief 최종 비용 맵을 퍼블리시합니다.
     */
    void publishPlanningMap() {
        grid_map_msgs::GridMap planning_map_msg;
        grid_map::GridMapRosConverter::toMessage(gridMap, planning_map_msg);
        grid_map_pub_.publish(planning_map_msg);
    }
};


class OdomHandler {
public:
    nav_msgs::Odometry receivedOdom;
    bool odom_ready_flag = false;
    double odom_position_x = 0.0, odom_position_y = 0.0;
    double odom_rotation_x = 0.0, odom_rotation_y = 0.0;
    double odom_rotation_z = 0.0, odom_rotation_w = 0.0;
    double odom_rotation_roll = 0.0, odom_rotation_pitch = 0.0, odom_rotation_yaw = 0.0;

    void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!odom_ready_flag) {
            receivedOdom = *msg;
            odom_position_x = receivedOdom.pose.pose.position.x;
            odom_position_y = receivedOdom.pose.pose.position.y;
            odom_rotation_x = receivedOdom.pose.pose.orientation.x;
            odom_rotation_y = receivedOdom.pose.pose.orientation.y;
            odom_rotation_z = receivedOdom.pose.pose.orientation.z;
            odom_rotation_w = receivedOdom.pose.pose.orientation.w;
            // Create a tf::Quaternion from the received quaternion
            tf::Quaternion q(odom_rotation_x, odom_rotation_y, odom_rotation_z, odom_rotation_w);
            tf::Matrix3x3 m(q);
            m.getRPY(odom_rotation_roll, odom_rotation_pitch, odom_rotation_yaw);  // Get roll, pitch, and yaw

            ROS_WARN(">>>>>>> Received [Odometry]:");
            ROS_INFO("[Odom] Position (x, y, z): %.2f, %.2f, %.2f", odom_position_x, odom_position_y, 0.0);
            ROS_WARN("-----------------------------------------------------------------------");
            odom_ready_flag = true;
        }
    }
}; /////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PlanningVisualizer {
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher path_pub_;
    ros::Publisher graph_pub_;
    ros::Publisher frontier_pub_;
    ros::Publisher frontier_pub2_;

    visualization_msgs::MarkerArray path_markers_;
    visualization_msgs::MarkerArray graph_markers_;
    visualization_msgs::MarkerArray frontier_markers_;
    visualization_msgs::MarkerArray frontier_markers2_;
    double marker_lifetime_ = 1.5;

    PlanningVisualizer(): nh_private_("~")
    {
        path_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("path_nodes", 10);
        graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("graph_nodes", 10);
        frontier_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("frontier_nodes", 10);
        frontier_pub2_ = nh_private_.advertise<visualization_msgs::MarkerArray>("frontier_nodes_local", 10);
    }

    void clearALL() {
        // cout markers size
        std::cout << "path_markers size: " << path_markers_.markers.size() << std::endl;
        std::cout << "graph_markers size: " << graph_markers_.markers.size() << std::endl;
        std::cout << "frontier_markers size: " << frontier_markers_.markers.size() << std::endl;
        std::cout << "frontier_markers2 size: " << frontier_markers2_.markers.size() << std::endl;

        if(path_markers_.markers.size()>0) {
            for (std::size_t i = 0; i < path_markers_.markers.size(); i++) {
                path_markers_.markers[i].action = visualization_msgs::Marker::DELETE;
            }
            path_pub_.publish(path_markers_);
        }
        path_markers_.markers.clear();

        if(graph_markers_.markers.size()>0) {
            for (std::size_t j = 0; j < graph_markers_.markers.size(); j++) {
                graph_markers_.markers[j].action = visualization_msgs::Marker::DELETE;
            }
            graph_pub_.publish(graph_markers_);
        }
        graph_markers_.markers.clear();

        if(frontier_markers_.markers.size()>0) {
            for (std::size_t k = 0; k < frontier_markers_.markers.size(); k++) {
                frontier_markers_.markers[k].action = visualization_msgs::Marker::DELETE;
            }
            frontier_pub_.publish(frontier_markers_);
        }
        frontier_markers_.markers.clear();

        if(frontier_markers2_.markers.size()>0) {
            for (std::size_t k = 0; k < frontier_markers2_.markers.size(); k++) {
                frontier_markers2_.markers[k].action = visualization_msgs::Marker::DELETE;
            }
            frontier_pub2_.publish(frontier_markers2_);
        }
        frontier_markers2_.markers.clear();
    }

    void visualizePath(std::vector<ob::State*>& path_states, double local_goal_x, double local_goal_y) {
        int counter = 0;
        visualization_msgs::Marker markerG;
        markerG.header.frame_id = "path_odom";
        markerG.header.stamp = ros::Time::now();
        markerG.ns = "goal";
        markerG.id = counter ++;
        markerG.type = visualization_msgs::Marker::SPHERE;
        markerG.action = visualization_msgs::Marker::ADD;
        markerG.pose.position.x = local_goal_x;
        markerG.pose.position.y = local_goal_y;
        markerG.pose.position.z = 1.1;
        markerG.pose.orientation.x = 0.0;
        markerG.pose.orientation.y = 0.0;
        markerG.pose.orientation.z = 0.0;
        markerG.pose.orientation.w = 1.0;
        markerG.scale.x = 1.5;
        markerG.scale.y = 1.5;
        markerG.scale.z = 1.5;
        markerG.color.a = 1.0;
        markerG.color.r = 0.0;
        markerG.color.g = 1.0;
        markerG.color.b = 0.0;
        markerG.lifetime = ros::Duration(marker_lifetime_);
        path_markers_.markers.push_back(markerG);
        std::size_t path_length_ = path_states.size();
        for (std::size_t node_i = 0; node_i < path_length_; ++node_i) {
            auto rstate = path_states[node_i]->as<ob::RealVectorStateSpace::StateType>();
            double x = rstate->values[0], y = rstate->values[1];
            visualization_msgs::Marker marker;
            marker.header.frame_id = "path_odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "path";
            marker.id = counter++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 1.1;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.lifetime = ros::Duration(marker_lifetime_);

            if (node_i == 0) {
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else if (node_i == path_length_ - 1) {
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            else{
                double rr = 1.0 - 1.0 * ((double)node_i / (path_length_ - 1));
                double bb = 1.0 * ((double)node_i / (path_length_ - 1));
                marker.color.a = 1.0;
                marker.color.r = rr;
                marker.color.g = 0.0;
                marker.color.b = bb;
            }
            path_markers_.markers.push_back(marker);
        }
        path_pub_.publish(path_markers_);
    }

    void visualizeGraph(ob::PlannerData& pldata) {
        double line_scale = 5.0;
//        graph_pub_.publish(graph_markers_);
        int counter = 0;
        visualization_msgs::Marker edge_marker;
        // Configure edge marker properties
        edge_marker.header.frame_id = "path_odom";
        edge_marker.header.stamp = ros::Time::now();
        edge_marker.ns = "edges";
        edge_marker.id = counter++;
        edge_marker.type = visualization_msgs::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::Marker::ADD;
        edge_marker.scale.x = 0.009 * line_scale; // Width of the lines
        edge_marker.color.a = 1.0; // Don't forget to set the alpha!
        edge_marker.color.r = 0.0; // Color red for edges
        edge_marker.color.g = 0.0;
        edge_marker.color.b = 0.0;
        edge_marker.lifetime = ros::Duration(marker_lifetime_);
        // Adding edges
        for (unsigned int v = 0; v < pldata.numVertices(); ++v) {
            // std::cout << "index: " << v << "  position: " << pldata.getVertex(v).getState()->as<ob::RealVectorStateSpace::StateType>()->values[0] << ", " << pldata.getVertex(v).getState()->as<ob::RealVectorStateSpace::StateType>()->values[1] << std::endl;
            std::vector<unsigned int> edgeList;
            pldata.getEdges(v, edgeList);
            // std::cout << "edgeList size: " << edgeList.size() << std::endl;
            const ob::RealVectorStateSpace::StateType *fromState = pldata.getVertex(v).getState()->as<ob::RealVectorStateSpace::StateType>();
            for (unsigned int i = 0; i < edgeList.size(); ++i) {
                const ob::RealVectorStateSpace::StateType *toState = pldata.getVertex(edgeList[i]).getState()->as<ob::RealVectorStateSpace::StateType>();
                geometry_msgs::Point p_from, p_to;
                p_from.x = fromState->values[0] ;
                p_from.y = fromState->values[1] ;
                p_from.z = 1.1;
                p_to.x = toState->values[0] ;
                p_to.y = toState->values[1] ;
                p_to.z = 1.1;
                // Add points to the edge_marker
                edge_marker.points.push_back(p_from);
                edge_marker.points.push_back(p_to);
            }
        }
        // Don't forget to add the edge_marker to your graph_markers array
        graph_markers_.markers.push_back(edge_marker);
        graph_pub_.publish(graph_markers_);
    }

    void visualizeFrontiers(std::vector<std::pair<double, double>>& frontier_positions) {
        int counter = 0;
        for (const auto& frontier : frontier_positions) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "path_odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "frontier";
            marker.id = counter++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = frontier.first;
            marker.pose.position.y = frontier.second;
            marker.pose.position.z = 1.1;
            marker.scale.x = 1.5;
            marker.scale.y = 1.5;
            marker.scale.z = 2.5;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.7;
            marker.color.b = 1.0;
            marker.lifetime = ros::Duration(marker_lifetime_);
            frontier_markers_.markers.push_back(marker);
        }
        frontier_pub_.publish(frontier_markers_);
    }
    void visualizeFrontiers_local(std::vector<std::pair<double, double>>& frontier_positions) {
        int counter = 0;
        for (const auto& frontier : frontier_positions) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = robot_frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "frontier";
            marker.id = counter++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = frontier.first;
            marker.pose.position.y = frontier.second;
            marker.pose.position.z = 1.1;
            marker.scale.x = 1.1;
            marker.scale.y = 1.1;
            marker.scale.z = 3.5;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.7;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration(marker_lifetime_);
            frontier_markers2_.markers.push_back(marker);
        }
        frontier_pub2_.publish(frontier_markers2_);
    }

}; /////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Frontier search //

class DBSCAN {
public:
    DBSCAN(double eps, int minPts) : eps_(eps), minPts_(minPts) {}

    std::vector<int> fit(const std::vector<std::vector<double>>& data) {
        int n = data.size();
        std::vector<int> labels(n, -1);
        int clusterId = 0;

        for (int i = 0; i < n; ++i) {
            if (labels[i] != -1) continue;

            std::vector<int> neighbors = regionQuery(data, i);

            if (neighbors.size() < minPts_) {
                labels[i] = -1;
            } else {
                expandCluster(data, labels, i, neighbors, clusterId);
                ++clusterId;
            }
        }

        return labels;
    }

private:
    double eps_;
    int minPts_;

    void expandCluster(const std::vector<std::vector<double>>& data, std::vector<int>& labels, int pointIdx, std::vector<int>& neighbors, int clusterId) {
        labels[pointIdx] = clusterId;
        for (std::size_t i = 0; i < neighbors.size(); ++i) {
            int neighborIdx = neighbors[i];

            if (labels[neighborIdx] == -1) {
                labels[neighborIdx] = clusterId;
            }

            if (labels[neighborIdx] != -1) continue;

            labels[neighborIdx] = clusterId;
            std::vector<int> newNeighbors = regionQuery(data, neighborIdx);

            if (newNeighbors.size() >= minPts_) {
                neighbors.insert(neighbors.end(), newNeighbors.begin(), newNeighbors.end());
            }
        }
    }

    std::vector<int> regionQuery(const std::vector<std::vector<double>>& data, int pointIdx) {
        std::vector<int> neighbors;

        for (std::size_t i = 0; i < data.size(); ++i) {
            if (euclideanDistance(data[pointIdx], data[i]) <= eps_) {
                neighbors.push_back(i);
            }
        }

        return neighbors;
    }
    double euclideanDistance(const std::vector<double>& point1, const std::vector<double>& point2) {
        double dx = point1[0] - point2[0];
        double dy = point1[1] - point2[1];
        return std::sqrt(dx * dx + dy * dy);
    }
};

class FrontierVertexExtractor {
public:
    FrontierVertexExtractor(double eps, int min_samples)
            : eps_(eps), min_samples_(min_samples) {}

    // Main function to extract representative vertices (x, y)
    std::vector<std::pair<double, double>> getFrontiers(ob::PlannerData& pldata) {
        std::vector<std::vector<double>> vertex_data = extractVertexData(pldata);
        std::vector<int> hullVertices = findConvexHull(vertex_data);
        std::vector<int> frontierVertices = findNearbyVertices(vertex_data, hullVertices, 0.3);

        // Use DBSCAN to cluster frontier vertices
        std::vector<std::vector<double>> frontier_positions = getFrontierPositions(vertex_data, frontierVertices);
        std::vector<int> labels = performDBSCAN(frontier_positions);

        // Group vertices by cluster and find representatives
        return computeFrontiers(vertex_data, frontierVertices, labels);
    }

private:
    double eps_;
    int min_samples_;

    // Function to extract vertex data from pldata (using RealVectorStateSpace)
    std::vector<std::vector<double>> extractVertexData(ob::PlannerData& pldata) {
        std::vector<std::vector<double>> vertex_data;
        for (std::size_t i = 0; i < pldata.numVertices(); ++i) {
            const auto* rstate = pldata.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>();
            vertex_data.push_back({rstate->values[0], rstate->values[1]}); // Accessing x and y from RealVectorStateSpace
        }
        return vertex_data;
    }

    // Function to get frontier positions from vertex data and frontier vertices
    std::vector<std::vector<double>> getFrontierPositions(const std::vector<std::vector<double>>& vertex_data, const std::vector<int>& frontierVertices) {
        std::vector<std::vector<double>> frontier_positions;
        for (int i : frontierVertices) {
            frontier_positions.push_back({vertex_data[i][0], vertex_data[i][1]});
        }
        return frontier_positions;
    }

    // Perform DBSCAN clustering
    std::vector<int> performDBSCAN(const std::vector<std::vector<double>>& frontier_positions) {
        DBSCAN dbscan(eps_, min_samples_);
        return dbscan.fit(frontier_positions);
    }

    // Function to compute representative vertices from clusters
    std::vector<std::pair<double, double>> computeFrontiers(
            const std::vector<std::vector<double>>& vertex_data,
            const std::vector<int>& frontierVertices,
            const std::vector<int>& labels) {

        std::map<int, std::vector<int>> clusters;
        std::vector<std::pair<double, double>> representativePositions;

        // Group vertices by cluster
        for (int i = 0; i < labels.size(); ++i) {
            if (labels[i] != -1) {
                clusters[labels[i]].push_back(frontierVertices[i]);
            }
        }

        // Compute representative for each cluster
        for (const auto& cluster : clusters) {
            int representative_idx = findClusterRepresentative(cluster.second, vertex_data);
            representativePositions.push_back({vertex_data[representative_idx][0], vertex_data[representative_idx][1]});
        }

        return representativePositions;
    }

    // Function to find the representative vertex of a cluster (closest to the centroid)
    int findClusterRepresentative(const std::vector<int>& cluster_indices, const std::vector<std::vector<double>>& vertex_data) {
        double x_sum = 0.0, y_sum = 0.0;
        for (int idx : cluster_indices) {
            x_sum += vertex_data[idx][0];
            y_sum += vertex_data[idx][1];
        }
        double x_avg = x_sum / cluster_indices.size();
        double y_avg = y_sum / cluster_indices.size();

        // Find the vertex closest to the centroid
        int representative_idx = cluster_indices[0];
        double min_dist = std::numeric_limits<double>::max();
        for (int idx : cluster_indices) {
            double dist = std::sqrt(std::pow(vertex_data[idx][0] - x_avg, 2) + std::pow(vertex_data[idx][1] - y_avg, 2));
            if (dist < min_dist) {
                min_dist = dist;
                representative_idx = idx;
            }
        }
        return representative_idx;
    }

    // Convex Hull functions (can use same ones from previous code)
    double crossProduct(const std::vector<double>& o, const std::vector<double>& a, const std::vector<double>& b) {
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    }

    double euclideanDistance(const std::vector<double>& p1, const std::vector<double>& p2) {
        return std::sqrt(std::pow(p1[0] - p2[0], 2) + std::pow(p1[1] - p2[1], 2));
    }

    std::vector<int> findConvexHull(const std::vector<std::vector<double>>& vertices) {
        int n = vertices.size(), k = 0;
        std::vector<int> hull(2 * n);
        std::vector<std::pair<std::vector<double>, int>> sortedVertices;

        for (int i = 0; i < n; ++i) {
            sortedVertices.push_back({vertices[i], i});
        }
        std::sort(sortedVertices.begin(), sortedVertices.end());

        for (int i = 0; i < n; ++i) {
            while (k >= 2 && crossProduct(vertices[hull[k - 2]], vertices[hull[k - 1]], sortedVertices[i].first) <= 0) {
                k--;
            }
            hull[k++] = sortedVertices[i].second;
        }

        for (int i = n - 1, t = k + 1; i >= 0; --i) {
            while (k >= t && crossProduct(vertices[hull[k - 2]], vertices[hull[k - 1]], sortedVertices[i].first) <= 0) {
                k--;
            }
            hull[k++] = sortedVertices[i].second;
        }

        hull.resize(k - 1);
        return hull;
    }

    std::vector<int> findNearbyVertices(const std::vector<std::vector<double>>& vertices, const std::vector<int>& hullVertices, double threshold) {
        std::vector<int> nearbyVertices;
        for (int i = 0; i < vertices.size(); ++i) {
            bool is_near = false;

            for (int j = 0; j < hullVertices.size(); ++j) {
                int next_j = (j + 1) % hullVertices.size();
                const std::vector<double>& p1 = vertices[hullVertices[j]];
                const std::vector<double>& p2 = vertices[hullVertices[next_j]];

                double distance = std::abs(crossProduct(p1, p2, vertices[i])) / euclideanDistance(p1, p2);

                if (distance <= threshold) {
                    is_near = true;
                    break;
                }
            }
            if (is_near) {
                nearbyVertices.push_back(i);
            }
        }
        return nearbyVertices;
    }
};

// Frontier search //


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class HanwhaStateValidator : public ob::StateValidityChecker {
    grid_map::GridMap& grid_map_;
    bool vaild_verbose_flag = true;
    bool vaild_verbose_flag_only_error = true;

    double cost_threshold_;
public:
    HanwhaStateValidator(const ob::SpaceInformationPtr& si, 
                        grid_map::GridMap& grid_map,
                        double cost_threshold)
                        : ob::StateValidityChecker(si), 
                        grid_map_(grid_map),
                        cost_threshold_(cost_threshold) {}

    // 상태의 유효성을 검증하는 함수
    bool isValid(const ob::State* state) const override {
        // 현재 상태가 유효한지 확인
        if (!this->isStateInsideMap(state)) {
            if (vaild_verbose_flag) {
                ROS_ERROR("[Valid] Error! State at position (%.2f, %.2f) is outside the valid map.",
                          state->as<ob::RealVectorStateSpace::StateType>()->values[0],
                          state->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            }
            return false;
        }
        // 원래 상태의 비용 계산
        double cost_val = this->getCostValue(state);
        if (std::isnan(cost_val)) {
            if (vaild_verbose_flag) {
                ROS_ERROR("[Valid] Error! Cost value is NaN for state at position (%.2f, %.2f).",
                          state->as<ob::RealVectorStateSpace::StateType>()->values[0],
                          state->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            }
            return false;
        }
        if (cost_val > cost_threshold_) {
            if (vaild_verbose_flag  && !vaild_verbose_flag_only_error) {
                ROS_WARN("[Valid] Cost value (%.2f) exceeds threshold (%.2f) for state at position (%.2f, %.2f).",
                          cost_val, cost_threshold_,
                          state->as<ob::RealVectorStateSpace::StateType>()->values[0],
                          state->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            }
            return false;
        }
        return true;
    }
    // 비용 계산 함수
    double getCostValue(const ob::State* state) const {
        double sample_x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        double sample_y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const grid_map::Position sample_position(sample_x, sample_y);
        return grid_map_.atPosition("planning_cost_map_updated", sample_position);
    }
    // 상태가 맵 내에 있는지 확인하는 함수
    bool isStateInsideMap(const ob::State* state) const {
        double sample_x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        double sample_y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const grid_map::Position sample_position(sample_x, sample_y);
        return grid_map_.isInside(sample_position);
    }
};

class HanwhaObjectiveFunction : public ob::StateCostIntegralObjective {
    grid_map::GridMap& grid_map_;

    bool cost_verbose_flag = true;
    bool cost_verbose_flag_only_error = true;
    double default_cost_value_ = 9999.0;

    bool use_radius_based_cost_;
    double robot_radius_;
    int num_samples_per_ring_;
    int num_rings_around_robot_;
public:
    HanwhaObjectiveFunction(const ob::SpaceInformationPtr& si,
                            grid_map::GridMap& grid_map,
                            double use_radius_based_cost,
                            double robot_radius,
                            double num_samples_around_robot,
                            double num_samples_inside_radius)
            : ob::StateCostIntegralObjective(si, true), 
            grid_map_(grid_map),
            use_radius_based_cost_(use_radius_based_cost),
            robot_radius_(robot_radius),
            num_samples_per_ring_(num_samples_around_robot),
            num_rings_around_robot_(num_samples_inside_radius){}
            
    ob::Cost stateCost(const ob::State* state) const override {

        if (use_radius_based_cost_) {
            ob::Cost worst_cost(default_cost_value_);

            double total_cost = 0.0;
            double cost_value;

            if (this->isStateInsideMap(state)) { // is inside 
                cost_value = this->getCostValue(state);
                // print cost value
                if (cost_verbose_flag && !cost_verbose_flag_only_error) {
                    ROS_INFO("[Cost] Cost value: %.2f for state at position (%.2f, %.2f).",
                             cost_value,
                             state->as<ob::RealVectorStateSpace::StateType>()->values[0],
                             state->as<ob::RealVectorStateSpace::StateType>()->values[1]);
                }

                if (std::isnan(cost_value)) {
                    if (cost_verbose_flag) ROS_WARN("[Cost] Error! NaN value.");
                    return worst_cost;
                }
                total_cost += cost_value;

                ob::State* sample_state = si_->allocState();
                for (int radius_level = 1; radius_level <= num_rings_around_robot_; ++radius_level) {
                    double sample_radius = robot_radius_ * (static_cast<double>(radius_level) / num_rings_around_robot_);
                    // closer ring more sample
                    int adapted_num_samples_per_ring_ = static_cast<int>(num_samples_per_ring_ * (((num_rings_around_robot_ + 1.0) - static_cast<double>(radius_level)) / num_rings_around_robot_));
                    for (int i = 0; i < adapted_num_samples_per_ring_; ++i) { 
                        double angle = 2 * M_PI * i / adapted_num_samples_per_ring_;
                        double sample_x = state->as<ob::RealVectorStateSpace::StateType>()->values[0] + sample_radius * cos(angle);
                        double sample_y = state->as<ob::RealVectorStateSpace::StateType>()->values[1] + sample_radius * sin(angle);
                        sample_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = sample_x;
                        sample_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = sample_y;
                        if (this->isStateInsideMap(sample_state)) {
                            cost_value = this->getCostValue(sample_state);
                            if (std::isnan(cost_value)) {
                                if (cost_verbose_flag) ROS_WARN("[Cost] Error! NaN value.");
                                si_->freeState(sample_state);
                                return worst_cost;
                            }
                            total_cost += cost_value;
                        } else {
                            if (cost_verbose_flag) ROS_ERROR("[Cost] Error! State is outside the valid map.");
                            si_->freeState(sample_state);
                            return worst_cost;
                        }
                    }
                }
                si_->freeState(sample_state);
                return ob::Cost(total_cost);
            } else { // is outside map
                if (cost_verbose_flag) ROS_ERROR("[Cost] Error! State is outside the valid map.");
                return worst_cost;
            }

        } else {
            ob::Cost worst_cost(default_cost_value_);
            if (this->isStateInsideMap(state)) {
                double cost_value = this->getCostValue(state);
                if (std::isnan(cost_value)) {
                    if (cost_verbose_flag) ROS_WARN("[Cost] Error! NaN value.");
                    return worst_cost;
                }
                return ob::Cost(cost_value);
            } else {
                if (cost_verbose_flag) ROS_ERROR("[Cost] Error! State is outside the valid map.");
                return worst_cost;
            }
        }
    }

    double getCostValue(const ob::State* state) const {
        double sample_x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        double sample_y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const grid_map::Position sample_position(sample_x, sample_y);
        return grid_map_.atPosition("planning_cost_map_updated", sample_position);
    }

    bool isStateInsideMap(const ob::State* state) const {
        double sample_x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        double sample_y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const grid_map::Position sample_position(sample_x, sample_y);
        return grid_map_.isInside(sample_position);
    }
};


ob::OptimizationObjectivePtr HanwhaBalancedObjective(const ob::SpaceInformationPtr& si, 
                                                     grid_map::GridMap& grid_map, 
                                                     bool use_radius_based_cost, double robot_radius, int num_samples_per_ring, int num_rings_around_robot)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr hanwhaObj(new HanwhaObjectiveFunction(si, 
                                                                       grid_map,
                                                                       use_radius_based_cost,
                                                                       robot_radius,
                                                                       num_samples_per_ring,
                                                                       num_rings_around_robot));

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
     opt->addObjective(lengthObj, 5.0); // cost scale: 10
     opt->addObjective(hanwhaObj, 15.0 ); // cost scale (best solution): 4.0
    return ob::OptimizationObjectivePtr(opt);
}

class HanwhaSamplableGoalRegion : public ob::GoalSampleableRegion {
    double goal_threshold_;
    double local_goal_x_, local_goal_y_;
public:
    HanwhaSamplableGoalRegion(const ob::SpaceInformationPtr& si, double local_goal_x, double local_goal_y, double goal_threshold)
            : ob::GoalSampleableRegion(si), local_goal_x_(local_goal_x), local_goal_y_(local_goal_y), goal_threshold_(goal_threshold) {
        // Constructor: local_goal_x, local_goal_y 설정 및 goal_threshold 초기화
    }

    // Goal까지의 거리를 계산하는 함수
    double distanceGoal(const ob::State* state) const override {
        const auto* real_state = state->as<ob::RealVectorStateSpace::StateType>();
        double x = real_state->values[0];
        double y = real_state->values[1];

        // Euclidean 거리 계산
        double distance = std::sqrt(std::pow(x - local_goal_x_, 2) + std::pow(y - local_goal_y_, 2));

        // 거리 반환: goal_threshold를 넘으면 distance 반환, 아니면 0 반환
        return distance > goal_threshold_ ? distance : 0.0;
    }

    // Goal 영역 내에서 목표 상태를 샘플링하는 함수
    void sampleGoal(ob::State* state) const override {
        auto rng = std::make_shared<ompl::RNG>();
        auto real_state = state->as<ob::RealVectorStateSpace::StateType>();

        // 무작위 각도와 반경 내 거리 생성
        double angle = rng->uniformReal(0, 2 * M_PI); // 0 ~ 360도 각도
        double distance = rng->uniformReal(0, goal_threshold_); // goal_threshold 내 무작위 거리

        // 무작위 각도와 거리로 목표 상태 설정
        real_state->values[0] = local_goal_x_ + distance * cos(angle);
        real_state->values[1] = local_goal_y_ + distance * sin(angle);
    }

    // 최대 샘플 수 반환
    unsigned int maxSampleCount() const override {
        return 50;
    }
};

// ██████╗ ██╗      █████╗ ███╗   ██╗███╗   ██╗██╗███╗   ██╗ ██████╗     
// ██╔══██╗██║     ██╔══██╗████╗  ██║████╗  ██║██║████╗  ██║██╔════╝     
// ██████╔╝██║     ███████║██╔██╗ ██║██╔██╗ ██║██║██╔██╗ ██║██║  ███╗    
// ██╔═══╝ ██║     ██╔══██║██║╚██╗██║██║╚██╗██║██║██║╚██╗██║██║   ██║    
// ██║     ███████╗██║  ██║██║ ╚████║██║ ╚████║██║██║ ╚████║╚██████╔╝    
// ╚═╝     ╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝  ╚═══╝╚═╝╚═╝  ╚═══╝ ╚═════╝     
                                                                      
class PathPlanner {
private:
    // ROS Publisher for Combined Path and Odometry
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher path_with_odom_pub_;

    ob::PlannerPtr planner_;
    og::SimpleSetupPtr ss_;
    ob::StateSpacePtr space_;
    PlanningVisualizer visualizer_;

    grid_map::GridMap grid_map_;
    double odom_position_x_ = 0.0, odom_position_y_ = 0.0;
    double prev_odom_position_x_ = 0.0, prev_odom_position_y_ = 0.0;
    double odom_rotation_yaw_ = 0.0, prev_odom_rotation_yaw_ = 0.0;
    double global_goal_x_, global_goal_y_;
    double local_goal_x_, local_goal_y_;

    bool is_planning_debug_ = false;
    double planning_time_s_ = 0.2;
//    std::string planner_name_ = "RRTstar";
//    std::string planner_name_ = "PRMstar";
//    std::string planner_name_ = "BITstar";
    std::string planner_name_ = "AITstar";

    bool interpolate_flag_ = true;

    bool is_pub_graph_ = false;
    bool planning_verbose_flag = false;

    double planning_map_size_ = 17;
    const double X_MIN_ = -planning_map_size_, X_MAX_ = planning_map_size_;
    const double Y_MIN_ = -planning_map_size_, Y_MAX_ = planning_map_size_;
    const double goal_projection_radius_ = planning_map_size_;

    std::vector<std::tuple<double, double>> prev_solpath_;
    bool initialize_with_prev_solpath_;

    bool is_linesearch_valid_goal_ = true;

    double robot_radius_ = 0.5;
    double goal_threshold_ = robot_radius_ * 2;

    // StateValidityChecker
    double SVC_cost_threshold_ = 0.8;

    // ObjectiveFunction
    bool OF_use_radius_based_cost_ = false;
    int OF_num_samples_per_ring_ = 6;
    int OF_num_rings_around_robot_ = 2;

    // Frontier Search
    bool enable_frontier_search_ = false;
    FrontierVertexExtractor extractor_;
    std::vector<std::pair<double, double>> frontier_set_;

    bool is_first_planning_ = true;

public:
    PathPlanner(): nh_private_("~"), 
                   extractor_(10.0, 5) // Initialize extractor with eps (클러스터로 고려한 포인트들간의 최대거리) and min_samples (클러스터를 구성하는데 필요한 샘플수)
    {
        // 상태 공간 정의
        space_ = std::make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, X_MIN_);
        bounds.setHigh(0, X_MAX_);
        bounds.setLow(1, Y_MIN_);
        bounds.setHigh(1, Y_MAX_);
        space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        ss_ = std::make_shared<og::SimpleSetup>(space_);

        if (planner_name_ == "RRTstar" || planner_name_ == "AITstar"){
            initialize_with_prev_solpath_ = true;
            ROS_WARN("[ERROR] RRTstar and AITstar support initialize_with_prev_solpath_.");
        } else {
            initialize_with_prev_solpath_ = false;
        }

        path_with_odom_pub_ = nh_.advertise<planning_module::PathData>("/path_with_odom", 10);

        loadROSParams();
    }

    void loadROSParams(){
        // OMPL 로그 레벨 설정
        nh_private_.param("is_planning_debug", is_planning_debug_, false);
        if (is_planning_debug_){
            ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_INFO);
        }else{
            ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);
        }

        if (is_planning_debug_){
            is_pub_graph_ = true;
        }else{
            is_pub_graph_ = false;
        }

        nh_private_.param("planning_time_s", planning_time_s_, 1.0);

        ROS_INFO("[planner] Planning time budget: %.2f sec", planning_time_s_);
    }

    void configureAndSetGeoPlanner(const std::string& plannerName) {
        // 이전 초기 경로 입력 준비
        if (initialize_with_prev_solpath_ && (prev_solpath_.size() > 0)){
            ROS_WARN("\033[1;93mInitialization with Prev sol. - plannerName: %s, prev_solpath_.size(): %.2i", plannerName.c_str(), prev_solpath_.size());
            for(std::size_t i = 0 ; i < prev_solpath_.size() ; i++){
                double delta_yaw = odom_rotation_yaw_ - prev_odom_rotation_yaw_;
                Point rev_rot_point = this->rotatePoint(std::get<0>(prev_solpath_[i]), std::get<1>(prev_solpath_[i]), -delta_yaw);
                std::get<0>(prev_solpath_[i]) = rev_rot_point.x - (odom_position_x_ - prev_odom_position_x_);
                std::get<1>(prev_solpath_[i]) = rev_rot_point.y - (odom_position_y_ - prev_odom_position_y_);
            }
        }
        
        if (plannerName == "RRTstar") {
            auto rrtstar = std::make_shared<og::RRTstar>(ss_->getSpaceInformation());
            rrtstar->setRange(3.0); // Adjust range for extending the tree
            // rrtstar->setKNearest(true); // Use k-nearest neighbor search
            rrtstar->setRewireFactor(5.0);  // initial radius!
            // rrtstar->setGoalBias(0.1); // Adjust goal bias
            // rrtstar->setTreePruning(true);
            if (initialize_with_prev_solpath_){
                rrtstar->setPrevSolPath(prev_solpath_);
            }
            planner_ = rrtstar;
        } else if (plannerName == "PRMstar") {
            planner_ = std::make_shared<og::PRMstar>(ss_->getSpaceInformation());
        } else if (plannerName == "LazyPRMstar") {
            planner_ = std::make_shared<og::LazyPRMstar>(ss_->getSpaceInformation());
        } else if (plannerName == "BITstar") {
            auto bitstar = std::make_shared<og::BITstar>(ss_->getSpaceInformation());
            bitstar->setUseKNearest(true);
//        bitstar->setRewireFactor(1.0);
            bitstar->setSamplesPerBatch(100);
            bitstar->setPruning(false);
            bitstar->setConsiderApproximateSolutions(false);
            planner_ = bitstar;
        } else if (plannerName == "AITstar") {
            auto aitstar = std::make_shared<og::AITstar>(ss_->getSpaceInformation());
            aitstar->setUseKNearest(true);
            aitstar->setBatchSize(100);
//            aitstar->setRewireFactor(3.0);
            aitstar->enablePruning(true);
            aitstar->trackApproximateSolutions(false);
            planner_ = aitstar;
            if (initialize_with_prev_solpath_){
                aitstar->setPrevSolPath(prev_solpath_);
            }
        } else {
            std::cerr << "Unknown planner specified: " << plannerName << std::endl;
            return;
        }
        ss_->setPlanner(planner_);
        if (planning_verbose_flag){
            std::cout << "[INFO] Planner settings ------------------" << std::endl;
            planner_->printSettings(std::cout);
            std::cout << "------------------------------------------" << std::endl;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool planPath(grid_map::GridMap& grid_map, double odom_x, double odom_y, double odom_yaw, double global_goal_x, double global_goal_y) {

        loadROSParams();

        grid_map_ = grid_map;
        odom_position_x_ = odom_x;
        odom_position_y_ = odom_y;
        odom_rotation_yaw_ = odom_yaw;
        global_goal_x_ = global_goal_x;
        global_goal_y_ = global_goal_y;

        // 상태 유효성 검사기 설정 (임시로 항상 유효한 상태로 설정)
        ss_->setStateValidityChecker(ob::StateValidityCheckerPtr(new HanwhaStateValidator(ss_->getSpaceInformation(), 
                                                                                        grid_map_,
                                                                                        SVC_cost_threshold_)));
        ss_->getSpaceInformation()->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(ss_->getSpaceInformation()));
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.005); // (15m map) 0.005 ==> 0.21 m

        // 최적화 목표 설정
        auto objective = HanwhaBalancedObjective(ss_->getSpaceInformation(), 
                                                 grid_map_,
                                                 OF_use_radius_based_cost_,
                                                 robot_radius_,
                                                 OF_num_samples_per_ring_,
                                                 OF_num_rings_around_robot_);
        ss_->getProblemDefinition()->setOptimizationObjective(objective);
        this->setPlanningGoal(ss_->getSpaceInformation());

        // 시작 상태 및 목표 상태 설정
        ob::ScopedState<ob::RealVectorStateSpace> start(space_);
        start->values[0] = 0.0;
        start->values[1] = 0.0;
        ss_->setStartState(start);

        ob::ScopedState<ob::RealVectorStateSpace> goal(space_);
        goal->values[0] = local_goal_x_;
        goal->values[1] = local_goal_y_;
        ss_->setGoalState(goal);

        ss_->setGoal(std::make_shared<HanwhaSamplableGoalRegion>(ss_->getSpaceInformation(), local_goal_x_, local_goal_y_, goal_threshold_));

        // 경로 계획기 설정
        configureAndSetGeoPlanner(planner_name_);
        ss_->setup();

        if (planning_verbose_flag) {
            std::cout << "[planner] StateValidityCheckingResolution: "
                      << ss_->getSpaceInformation()->getStateValidityCheckingResolution() << std::endl;
            std::cout << "[planner] LongestValidSegmentLength: "
                      << ss_->getSpaceInformation()->getStateSpace()->getLongestValidSegmentLength() << std::endl;
        }

        // 경로 계획 수행
        ob::PlannerStatus solved = ss_->solve(planning_time_s_);
        visualizer_.clearALL();
        if (solved) {
            ROS_INFO("\033[1;92mSolved!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! with planning time %.2f sec", planning_time_s_);

            // 원래 경로 얻기
            og::PathGeometric originalPath = ss_->getSolutionPath();
            double originalCost = originalPath.cost(objective).value();
            ROS_WARN("[Simplification] Original path cost: %.2f", originalCost);

            // 경로 복사본 생성 (단순화에 사용할 경로)
            og::PathGeometric simplifiedPath(originalPath);
            og::PathSimplifier simplifier(ss_->getSpaceInformation());
            simplifier.simplifyMax(simplifiedPath);
            simplifier.smoothBSpline(simplifiedPath);
            double simplifiedCost = simplifiedPath.cost(objective).value();
            ROS_WARN("[Simplification] Simplified path cost: %.2f", simplifiedCost);

            // 비용 비교 및 결정
            if (simplifiedCost <= originalCost)
            {
                ROS_INFO("\033[1;92m[Simplification] Simplified path accepted. (cost did not increase)");
                ss_->getSolutionPath() = simplifiedPath;
            }
            else
            {
                ROS_WARN("\033[1;95m[Simplification] Simplified path rejected.");
            }

            // 최종 경로 출력
            og::PathGeometric finalPath = ss_->getSolutionPath();
            if (interpolate_flag_){
                finalPath.interpolate(30);
            }
            std::vector<ob::State*> path_states = finalPath.getStates();

            //print path length and cost
            ROS_INFO("\033[1;92m[Solution] Path length: %.2f, Path cost: %.2f", finalPath.length(), finalPath.cost(objective).value());
            visualizer_.visualizePath(path_states, local_goal_x_, local_goal_y_);

            if (is_pub_graph_) {
                ob::PlannerData pldata(ss_->getSpaceInformation()); // [TODO] mem leaking checking
                ss_->getPlannerData(pldata);

                // Edge 개수가 0인 vertex들을 삭제하기 위해 새로운 PlannerData 생성
                unsigned int num_vertices = pldata.numVertices();

                // edge가 없는 vertex들의 인덱스를 저장할 집합
                std::unordered_set<unsigned int> vertices_to_delete;

                // 모든 vertex를 순회하면서 edge 개수가 0인 vertex의 인덱스를 저장
                for (unsigned int v = 0; v < num_vertices; ++v) {
                    std::vector<unsigned int> edgeList;
                    pldata.getEdges(v, edgeList);
                    if (edgeList.size() == 0) {
                        vertices_to_delete.insert(v);
                    }
                }

                // vertex를 삭제하기 위해 새로운 PlannerData 생성
                ob::PlannerData new_pldata(ss_->getSpaceInformation());

                // 기존의 vertex 중 edge가 있는 vertex만 새로운 PlannerData에 추가
                std::unordered_map<unsigned int, unsigned int> index_mapping; // 원래 인덱스 -> 새로운 인덱스
                unsigned int new_index = 0;
                for (unsigned int v = 0; v < num_vertices; ++v) {
                    if (vertices_to_delete.find(v) == vertices_to_delete.end()) {
                        const ob::PlannerDataVertex& vertex = pldata.getVertex(v);
                        new_pldata.addVertex(vertex);
                        index_mapping[v] = new_index;
                        new_index++;
                    }
                }

                // edge를 새로운 PlannerData에 추가
                for (unsigned int v = 0; v < num_vertices; ++v) {
                    if (vertices_to_delete.find(v) == vertices_to_delete.end()) {
                        std::vector<unsigned int> edgeList;
                        pldata.getEdges(v, edgeList);
                        unsigned int new_v = index_mapping[v];
                        for (unsigned int i = 0; i < edgeList.size(); ++i) {
                            unsigned int u = edgeList[i];
                            if (vertices_to_delete.find(u) == vertices_to_delete.end()) {
                                unsigned int new_u = index_mapping[u];
                                new_pldata.addEdge(new_v, new_u);
                            }
                        }
                    }
                }

                visualizer_.visualizeGraph(new_pldata);

                if(enable_frontier_search_){
                    std::cout << "[Frontier] Frontier search starts, pldata size: " << new_pldata.numVertices() << std::endl;
                    std::vector<std::pair<double, double>> frontier_set = extractor_.getFrontiers(new_pldata);
                    frontier_set_.clear();
                    frontier_set_.resize(frontier_set.size());
                    frontier_set_ = frontier_set;
                    visualizer_.visualizeFrontiers(frontier_set_);
                    std::cout << "[Frontier] Frontier set size: " << frontier_set_.size() << std::endl;
                }
            }

            if (initialize_with_prev_solpath_) {
                prev_solpath_.clear();
                for (std::size_t node_i = 0; node_i < path_states.size(); ++node_i) {
                    auto r2state = path_states[node_i]->as<ob::RealVectorStateSpace::StateType>();
                    double x = r2state->values[0], y = r2state->values[1];
                    prev_solpath_.push_back(std::make_tuple(x, y));
                }
            }

            // ** Combined Path and Odometry 메시지 퍼블리시 시작 **
            planning_module::PathData combined_msg;
            // Path Nodes 설정
            combined_msg.path_nodes = visualizer_.path_markers_;
            // Odometry 정보 설정
            combined_msg.path_odom.header.stamp = ros::Time::now();
            combined_msg.path_odom.header.frame_id = "none"; 
            combined_msg.path_odom.pose.pose.position.x = odom_position_x_;
            combined_msg.path_odom.pose.pose.position.y = odom_position_y_;
            combined_msg.path_odom.pose.pose.position.z = 0.0;
            tf::Quaternion q = tf::createQuaternionFromYaw(odom_rotation_yaw_);
            combined_msg.path_odom.pose.pose.orientation.x = q.x();
            combined_msg.path_odom.pose.pose.orientation.y = q.y();
            combined_msg.path_odom.pose.pose.orientation.z = q.z();
            combined_msg.path_odom.pose.pose.orientation.w = q.w();
            path_with_odom_pub_.publish(combined_msg);

            prev_odom_position_x_ = odom_position_x_;
            prev_odom_position_y_ = odom_position_y_;
            prev_odom_rotation_yaw_ = odom_rotation_yaw_;

            // pldata.clear();
            // new_pldata.clear();
            planner_->clear();
            ss_->clear();
            return true;
        }
        else{
            ROS_ERROR("@@@@@@@@@@@@@@@@@@@@ Failed to find a path. @@@@@@@@@@@@@@@@@@@@");
            planner_->clear();
            ss_->clear();
            return false;
        }
    }

     void setPlanningGoal(const ob::SpaceInformationPtr& si_) {
         // 로컬 맵위로 투영된 목표
         Point proj_goal = this->projectToCircle(global_goal_x_ - odom_position_x_, global_goal_y_ - odom_position_y_);
         Point rotated_goal = this->rotatePoint(proj_goal.x, proj_goal.y, -odom_rotation_yaw_);
         local_goal_x_ = rotated_goal.x;
         local_goal_y_ = rotated_goal.y;

         if(is_linesearch_valid_goal_){
             // 초기 goal state 설정
             ob::State* local_goal_state = si_->allocState();
             local_goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = local_goal_x_;
             local_goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = local_goal_y_;

             // goal이 유효하지 않은 경우, 원점 방향으로 line search
             double search_step = 0.5; // 탐색 간격
             int max_search_steps = 20; // 최대 탐색 스텝 수
             bool found_valid_goal = false;

             for (int step = 0; step < max_search_steps; ++step) {
                 double cost_value = this->get_cost_value(local_goal_state);
                 ROS_INFO("Goal Search cost (%.2i iter): %.2f", step, cost_value);
                 if (!std::isnan(cost_value) && cost_value < SVC_cost_threshold_) {
                     found_valid_goal = true;
                     ROS_INFO("Valid goal found at: (%.2f, %.2f), cost: %.2f", local_goal_x_, local_goal_y_, cost_value);
                     break;
                 }else{
                     ROS_WARN("Invalid goal position. Searching for a valid goal position...");
                 }

                 // 원점 방향으로 선형 이동
                 local_goal_x_ -= search_step * (local_goal_x_ / sqrt(local_goal_x_ * local_goal_x_ + local_goal_y_ * local_goal_y_));
                 local_goal_y_ -= search_step * (local_goal_y_ / sqrt(local_goal_x_ * local_goal_x_ + local_goal_y_ * local_goal_y_));

                 local_goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = local_goal_x_;
                 local_goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = local_goal_y_;

                 if (step == max_search_steps - 1) {
                     ROS_WARN("Max search steps reached. Using last valid goal position.");
                 }
             }
             si_->freeState(local_goal_state);

             if (!found_valid_goal) {
                 ROS_ERROR("Failed to find a valid goal position within the search range.");
                 // 마지막으로 valid했던 위치나 원래 goal을 사용할 수 있음 (여기서는 원래 목표로 둡니다)
             }
         }
         ROS_INFO("\033[1;92m[Problem] Planning Goal: (%.2f, %.2f)", local_goal_x_, local_goal_y_);
     }


    Point transformFrontier(const std::pair<double, double>& frontier,
                            double prev_odom_x, double prev_odom_y, double prev_yaw,
                            double curr_odom_x, double curr_odom_y, double curr_yaw) {
        // 1. prev 오돔에서의 frontier 좌표를 현재 오돔으로 변환하기 위해 delta 계산
        double delta_x = frontier.first - prev_odom_x;
        double delta_y = frontier.second - prev_odom_y;

        // 2. prev 오돔과 현재 오돔 사이의 yaw 차이 보상 (회전 변환)
        double cos_yaw = cos(curr_yaw - prev_yaw);
        double sin_yaw = sin(curr_yaw - prev_yaw);
        double rotated_x = delta_x * cos_yaw - delta_y * sin_yaw;
        double rotated_y = delta_x * sin_yaw + delta_y * cos_yaw;

        // 3. 회전된 frontier 위치를 현재 오돔 위치로 이동 변환
        Point transformed_frontier;
        transformed_frontier.x = rotated_x + curr_odom_x;
        transformed_frontier.y = rotated_y + curr_odom_y;

        return transformed_frontier;
    }

    Point rotatePoint(double x, double y, double yaw) const {
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        Point rotated;
        rotated.x = x * cos_yaw - y * sin_yaw;
        rotated.y = x * sin_yaw + y * cos_yaw;
        return rotated;
    }

    Point projectToCircle(double x, double y) {
        double length = std::sqrt(x * x + y * y);
        Point projected;
        if (length == 0) {
            projected.x = goal_projection_radius_;
            projected.y = 0;
        } else if (length <= goal_projection_radius_) {
            projected.x = x;
            projected.y = y;
        } else {
            projected.x = (x / length) * goal_projection_radius_;
            projected.y = (y / length) * goal_projection_radius_;
        }
        return projected;
    }

    double get_cost_value(const ob::State* state) const {
        double sample_x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        double sample_y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const grid_map::Position sample_position(sample_x, sample_y);
        return grid_map_.atPosition("planning_cost_map_updated", sample_position);
    }

}; /////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ROSPlanningNode {
private:
    ros::NodeHandle nh_;
    GridMapHandler grid_map_handler_;
    OdomHandler odom_handler_;

    PathPlanner path_planner_;

    ros::Subscriber grid_map_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher planning_result_pub_;  // Planning 결과를 publish할 publisher
    ros::Publisher global_goal_pub_;      // 글로벌 목표 마커를 퍼블리시할 publisher 추가


    double global_goal_x_;
    double global_goal_y_;
public:
    ROSPlanningNode()
    {
        grid_map_sub_ = nh_.subscribe("/trip/trip_updated/terrain_local_gridmap", 1, &GridMapHandler::gridMapCallback, &grid_map_handler_);
        odom_sub_ = nh_.subscribe(odom_topic_name, 1, &OdomHandler::gpsCallback, &odom_handler_);
        planning_result_pub_ = nh_.advertise<std_msgs::Int32>("/planning_result", 1);  // Int32 메시지 타입으로 설정
        global_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("/global_goal", 1); // 마커 퍼블리셔 초기화
    }
    void spin() {
        while (ros::ok()) {
            ros::spinOnce();  // 메시지 처리

            // planning 결과를 Int로 표현
            std_msgs::Int32 planning_result_msg;

            if (grid_map_handler_.map_ready_flag && odom_handler_.odom_ready_flag) {
                //  Load global goal position
                if (!nh_.getParam("/global_goal_x", global_goal_x_) || !nh_.getParam("/global_goal_y", global_goal_y_)) {
                    global_goal_x_ = odom_handler_.odom_position_x;
                    global_goal_y_ = odom_handler_.odom_position_y;
                    ROS_ERROR("Parameter '/global_goal_x' or '/global_goal_y' not found. Using robot's current X position: %.6f, %.6f", global_goal_x_, global_goal_y_);

                    // 목표 미설정: 2로 publish
                    planning_result_msg.data = 2;
                    planning_result_pub_.publish(planning_result_msg);
                    continue;
                }else{
                    ROS_INFO("\033[1;96m[PLANNING START] Global Goal: x = %.2f, y = %.2f >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> \033[0m", global_goal_x_, global_goal_y_);
                }

                // 글로벌 목표를 큰 빨간색 구로 퍼블리시
                visualization_msgs::Marker goal_marker;
                goal_marker.header.frame_id = global_frame_id; // 적절한 프레임 설정
                goal_marker.header.stamp = ros::Time::now();
                goal_marker.ns = "global_goal";
                goal_marker.id = 0;
                goal_marker.type = visualization_msgs::Marker::SPHERE;
                goal_marker.action = visualization_msgs::Marker::ADD;
                goal_marker.pose.position.x = global_goal_x_;
                goal_marker.pose.position.y = global_goal_y_;
                goal_marker.pose.position.z = 0.0; // 필요에 따라 조정
                goal_marker.lifetime = ros::Duration(1.0);
                goal_marker.pose.orientation.x = 0.0;
                goal_marker.pose.orientation.y = 0.0;
                goal_marker.pose.orientation.z = 0.0;
                goal_marker.pose.orientation.w = 1.0;
                goal_marker.scale.x = 4.0; // 구의 크기 조정
                goal_marker.scale.y = 4.0;
                goal_marker.scale.z = 4.0;
                goal_marker.color.a = 0.8; // 투명도
                goal_marker.color.r = 1.0; // 빨간색
                goal_marker.color.g = 0.0;
                goal_marker.color.b = 0.0;

                // 마커 퍼블리시
                global_goal_pub_.publish(goal_marker);
                
                bool planning_solved = path_planner_.planPath(grid_map_handler_.gridMap,
                                                              odom_handler_.odom_position_x,
                                                              odom_handler_.odom_position_y,
                                                              odom_handler_.odom_rotation_yaw,
                                                              global_goal_x_, global_goal_y_);

                // 경로 계획 성공 여부에 따라 메시지 설정
                if (planning_solved) {
                    planning_result_msg.data = 1;  // 성공: 1
                } else {
                    planning_result_msg.data = 0;  // 실패: 0
                }
                planning_result_pub_.publish(planning_result_msg);

                grid_map_handler_.map_ready_flag = false;
                odom_handler_.odom_ready_flag = false;
                ROS_INFO("\033[1;96m<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< [PLANNING END]");
            }else{
                ros::Duration(0.01).sleep();
            }
        }
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");

    ROSPlanningNode node;
    node.spin();

    return 0;
}
