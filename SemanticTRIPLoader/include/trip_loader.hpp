#define PCL_NO_PRECOMPILE
#pragma once

// #include "trip/module/trip_completion.hpp"
#include "utils.hpp"

class TRIP_LOADER {
public:
    TRIP_LOADER() {
        std::cout << "\033[34;1m[TRIP-loader] Constructor\033[34;0m" << std::endl;
        m_world_map.reset(new pcl::PointCloud<PtTRIP>());
        
        m_local_map.reset(new pcl::PointCloud<PtTRIP>());
        m_local_map->clear();
        m_local_map->reserve(300000);
    };


    ~TRIP_LOADER() {
        std::cout << "\033[31;1m[TRIP-loader] Destructor\033[31;0m" << std::endl;
    };

    void setParams (const float &_resolution, const float &_radius) {
        m_resolution = _resolution;
        m_radius = _radius;
        m_buf_radius = _radius*sqrt(2.0);
        return;
    };  // end of setParams

    void setTerrainMap( const std::string &_dir, 
                        const std::string &_name, 
                        const std::string &_f_id) {

        std::string map_path = _dir + _name;
        std::cout << "Load from ... " << map_path << std::endl;
        pcl::io::loadPCDFile(map_path, *m_world_map);
        m_world_map->header.frame_id = _f_id;
        std::cout << "... Size: " << m_world_map->size() << std::endl;
        return;

        return;
    };  // end of loadTerrainMap

    void getLocalMap(   trip_types::StampedPose &_p, 
                        boost::shared_ptr<pcl::PointCloud<PtTRIP>> &_c) {
        
        // fitlering based on the pose
        m_local_map->clear();
        float _px = _p.pos.x();
        float _py = _p.pos.y();
        for (auto &p : m_world_map->points) {
            if (p.x > _px + m_buf_radius) continue;
            if (p.x < _px - m_buf_radius) continue;
            if (p.y > _py + m_buf_radius) continue;
            if (p.y < _py - m_buf_radius) continue;

            m_local_map->push_back(p);
        }

        // transform the local map
        Eigen::Matrix4f _tf = _p.getTransformMatrix().inverse();  // tf from local to global
        trip_types::transformTerrainMap<PtTRIP>(*m_local_map, *_c, _tf);
        *m_local_map = *_c;
        // filtering based on the radius
        _c->clear();
        _c->reserve(m_local_map->size());
        for (auto &p : m_local_map->points) {
            if (p.x > m_radius) continue;
            if (p.x < -m_radius) continue;
            if (p.y > m_radius) continue;
            if (p.y < -m_radius) continue;

            _c->push_back(p);
        }
        return;
    };  // end of getLocalMap

private:

    boost::shared_ptr<pcl::PointCloud<PtTRIP>> m_world_map;
    boost::shared_ptr<pcl::PointCloud<PtTRIP>> m_local_map;

    float m_resolution;
    float m_radius;
    float m_buf_radius;

};  // class TRIP_LOADER