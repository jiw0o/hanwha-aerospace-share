#define PCL_NO_PRECOMPILE
#include "utils_ros1.hpp"
#include "trip_loader.hpp"

using PtRaw = pcl::PointXYZI;

class TRIP_LOADER_ROS1 {
   public:
    TRIP_LOADER_ROS1() {
        setSub();
        setPub();
        m_trip_loader.reset(new TRIP_LOADER());
        cloud.reset(new pcl::PointCloud<PtTRIP>());
        rgb_cloud.reset(new pcl::PointCloud<PtColorTRIP>());
        grid_map.reset(new grid_map::GridMap());
        m_tf_br.reset(new tf::TransformBroadcaster());

        loadParam();

        label_color_[0] = cv::Vec3b(0, 0, 0);       // void
        label_color_[1] = cv::Vec3b(0, 102, 0);     // grass
        label_color_[2] = cv::Vec3b(0, 255, 0);     // tree
        label_color_[3] = cv::Vec3b(0, 128, 255);   // water
        label_color_[4] = cv::Vec3b(0, 0, 255);     // sky
        label_color_[5] = cv::Vec3b(64, 64, 64);    // asphalt
        label_color_[6] = cv::Vec3b(110, 22, 138);  // rubble
    };

    ~TRIP_LOADER_ROS1() {
        m_trip_loader->~TRIP_LOADER();
    };

   private:

    void loadParam() {
        // Load Parameters for TRIP Data Field
        std::string m_trip_dir, 
                    m_map_name;

        m_nh.param<bool>        ("use_terrain_frame"        , m_use_terrain_frame   , true);
        m_nh.param<bool>        ("use_semantic_info"        , m_use_semantic_info   , true);
        m_nh.param<std::string> ("loader/io/map_dir"        , m_trip_dir            , "");
        m_nh.param<std::string> ("loader/io/map_name"       , m_map_name            , "");
        m_nh.param<std::string> ("loader/map/map_frame"     , m_map_frame_id        , "");
        m_nh.param<float>       ("loader/map/resolution"    , m_resolution          , 0.0);
        m_nh.param<float>       ("loader/map/local_radius"  , m_radius              , 0.0);
        std::cout << "Loaded map_dir: "     << m_trip_dir       << std::endl;
        std::cout << "Loaded map_name: "    << m_map_name       << std::endl;
        std::cout << "Loaded map_frame_id: "<< m_map_frame_id   << std::endl;
        std::cout << "Loaded resolution: "  << m_resolution     << std::endl;
        std::cout << "Loaded local_radius: "<< m_radius         << std::endl;
        if (m_use_terrain_frame) {
            std::cout << "Using terrain frame (roll and pitch is 0)" << std::endl;
        } else {
            std::cout << "Using local estiamted pose frame" << std::endl;
        }

        if (m_use_semantic_info) {
            std::cout << "Using semantic information" << std::endl;
        } else {
            std::cout << "Not using semantic information" << std::endl;
        }

        m_trip_loader->setTerrainMap(   m_trip_dir, 
                                        m_map_name, 
                                        m_map_frame_id);

        m_trip_loader->setParams(   m_resolution, 
                                    m_radius);

        float m_map_width, 
              m_map_length;
        m_map_width = m_radius * 2.0;
        m_map_length = m_radius * 2.0;
        utils_ros1::initTRIPGridmap(*grid_map, m_map_frame_id, m_map_width, m_map_length, m_resolution);

        m_init_odom = false;

        // get the custom field of the PtTRIP
        PtTRIP pt;
        std::cout << "Size of PtTRIP: " << sizeof(pt) << std::endl;
                


        return;
    };  // end of loadParam

    void setSub() {

        m_nh.param<std::string>("topic/pose", 
                                sub_topic_odom, 
                                "/nav_msg_odom");
        std::cout << "Subscribed topic: " << sub_topic_odom << std::endl;
        sub_odom = m_nh.subscribe<ROS1_ODOM>(sub_topic_odom, 1, &TRIP_LOADER_ROS1::callbackOdom, this);
    };  // end of setSub

    void setPub() {

        m_nh.param<std::string>("topic/terrain_local_ptcloud", 
                                pub_topic_terrain_local_ptcloud, 
                                "/trip/trip_updated/terrain_local_ptcloud");
        
        m_nh.param<std::string>("topic/terrain_local_gridmap", 
                                pub_topic_terrain_local_gridmap, 
                                "/trip/trip_updated/terrain_local_gridmap");
        std::cout << "Published ptcloud topic: " << pub_topic_terrain_local_ptcloud << std::endl;
        std::cout << "Published gridmap topic: " << pub_topic_terrain_local_gridmap << std::endl;

        std::string pub_topic_odom_grav = sub_topic_odom + "_grav";
        pub_odom = m_nh.advertise<ROS1_ODOM>(pub_topic_odom_grav, 1);
        pub_terrain_local_ptcloud = m_nh.advertise<ROS1_PTCLOUD>(pub_topic_terrain_local_ptcloud, 1);
        pub_terrain_local_gridmap = m_nh.advertise<ROS1_GRIDMAP>(pub_topic_terrain_local_gridmap, 1);

        if (m_use_semantic_info) {
            pub_terrain_local_color_ptcloud = m_nh.advertise<ROS1_PTCLOUD>(pub_topic_terrain_local_ptcloud+"_colored", 1);
        }
    };  // end of setPub

private:

    void callbackOdom(const ROS1_ODOM::ConstPtr &_msg) {
        if (!m_init_odom) {
            if (m_map_frame_id != _msg->header.frame_id) {
                std::cout << "\033[31;1mError: Odom frame_id should be same as map_frame_id\033[31;0m" << std::endl;
                exit(-1);
            }
        }

        static boost::shared_ptr<trip_types::StampedPose> cur_pose(new trip_types::StampedPose);

        cur_pose->pos.x()  = _msg->pose.pose.position.x;
        cur_pose->pos.y()  = _msg->pose.pose.position.y;
        cur_pose->pos.z()  = _msg->pose.pose.position.z;
        cur_pose->quat.x() = _msg->pose.pose.orientation.x;
        cur_pose->quat.y() = _msg->pose.pose.orientation.y;
        cur_pose->quat.z() = _msg->pose.pose.orientation.z;
        cur_pose->quat.w() = _msg->pose.pose.orientation.w;
        cur_pose->stamp    = utils_ros1::get_time_sec(_msg->header.stamp);
        cur_pose->frame_id = _msg->header.frame_id;
        cur_pose->child_frame_id = _msg->child_frame_id;
        
        ROS1_HEADER local_header = _msg->header;
        local_header.frame_id = _msg->child_frame_id;


        if (m_use_terrain_frame) {
            static std::string terrain_frame_id = _msg->child_frame_id + "_terrain";
            local_header.frame_id = terrain_frame_id;
            // convert the cur_pose to cur_terrain_pose
            Eigen::Matrix4f tf_global2alocal = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f tf_base2aligned  = Eigen::Matrix4f::Identity();
            utils_ros1::getAlignMatfromStampedPose(*cur_pose, 
                                                    tf_global2alocal, 
                                                    tf_base2aligned);

            utils_ros1::convertTFEigen2StampedPose(tf_global2alocal, *cur_pose);

            // publish the tf

            tf::Transform tf_global2base;
            utils_ros1::convertOdom2Transform(*cur_pose, tf_global2base);


            utils_ros1::publishOdometry(_msg->header.stamp, 
                                        _msg->header.frame_id, 
                                        terrain_frame_id, 
                                        tf_global2base, 
                                        pub_odom, 
                                        *m_tf_br);

        }

        m_trip_loader->getLocalMap(*cur_pose, cloud);
        // utils_ros1::convertTRIPCloud2Gridmap(header.frame_id, *cloud, *grid_map, cur_pose->pos.x(), cur_pose->pos.y());
        utils_ros1::convertTRIPCloud2Gridmap(local_header.frame_id, *cloud, *grid_map, 0, 0);
        if (m_use_semantic_info) {
            utils_ros1::convertToColoredMap<PtTRIP, PtColorTRIP>(*cloud, *rgb_cloud, label_color_);
        }

        std::cout << "Publishing local ptcloud: " << local_header.frame_id << std::endl;
        std::cout << "Publishing cloud size: " << cloud->size() << std::endl;
        utils_ros1::publishCloud(local_header, cloud, pub_terrain_local_ptcloud);
        utils_ros1::publishTRIPGridmap(local_header, *grid_map, pub_terrain_local_gridmap);

        if(m_use_semantic_info) {
            utils_ros1::publishCloud(local_header, rgb_cloud, pub_terrain_local_color_ptcloud);
        }

        return;
    };  // end of callbackOdom

private:

    ros::NodeHandle m_nh;
    std::string m_map_frame_id;
    

    std::string sub_topic_odom, 
                pub_topic_terrain_local_ptcloud, 
                pub_topic_terrain_local_gridmap;

    float m_resolution, 
          m_radius;
    bool m_init_odom;
    bool m_use_terrain_frame;
    bool m_use_semantic_info;

    ros::Subscriber sub_odom;

    ros::Publisher  pub_odom;
    ros::Publisher  pub_terrain_local_ptcloud;
    ros::Publisher  pub_terrain_local_color_ptcloud;
    ros::Publisher  pub_terrain_local_gridmap;
    std::shared_ptr<tf::TransformBroadcaster> m_tf_br;


    boost::shared_ptr<TRIP_LOADER>  m_trip_loader;
    boost::shared_ptr<pcl::PointCloud<PtTRIP>> cloud;
    boost::shared_ptr<pcl::PointCloud<PtColorTRIP>> rgb_cloud;
    boost::shared_ptr<grid_map::GridMap> grid_map;

    std::map<int, cv::Vec3b> label_color_;

};  // class TRIP_LOADER_ROS1