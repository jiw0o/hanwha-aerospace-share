# Repo for Terrain Traversability Map Load and Publisher

## Main Contributors
    - Minh Oh: minho.oh@kaist.ac.kr
    - Seungjae Lee: sj98lee@kaist.ac.kr

## Build
```ruby
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Download the map
```ruby
cd ~/{$TRIP_PACKAGE}/map
chmod +x download_maps.sh
./download_maps.sh
```
-   Please make a "map" folder in the package first, and download the map file from the server
-   Make the directories as follows:
    - ./SemanticTRIPLoader
        - include
        - launch
        - map
            - trip_gazebo_map_20240913.pcd
            - semantic_terrain_map_has_gazebo_20241021.pcd
        - rviz
        - src

## How to run

```ruby
# on the Gazebo world
roslaunch trip_loader run_gazebo.launch

# on the real-world bunker data
roslaunch trip_loader run_bunker.launch
```
- Arguments
    - rviz:= true or false
        - true/false: turn on/off the rviz
    - use_terrain_frame:=true or false
        - true: publish the terrain map result on 4-dof without roll and pitch, which is parallal frame to gravity direction.
        - false: publish the terrain map result on 6-dof with roll and pitch, which is following the estimated robot pose.
    - use_semantic_info:=true or false
        - true: publish and show the semantic info of terrain map result by loading semantic terrain map.
        - false: publish the terrain map result without colors and labels.

- Parameters
    In the "run.launch" file,
    ```ruby
    <arg name="rviz" default="false" />
    <arg name="use_terrain_frame" default="true" />
    <arg name="use_semantic_info" default="true" />

    <arg name="map_dir" default="$(find trip_loader)/map/" />

    <arg if="$(arg use_semantic_info)" name="map_name" default="semantic_terrain_map_has_gazebo_20241021.pcd" />
    <arg unless="$(arg use_semantic_info)" name="map_name" default="terrain_map_has_gazebo_20241021.pcd" />

    <arg name="map_frame_id" default="map" />
    <arg name="resolution" default="0.2" />
    <arg name="radius" default="20.0" />

    <arg name="odom_topic" default="/odometry_gt" />
    <arg name="terrain_local_ptcloud" default="/trip/trip_updated/terrain_local_ptcloud" />
    <arg name="terrain_local_gridmap" default="/trip/trip_updated/terrain_local_gridmap" />
    ```
## Data Types for TRIP Results 
- grid_map_msgs::GridMap
    ```
    -    elevation: "height of the terrain"  
    -    min_elevation: "minimum height of the terrain"
    -    normal_x: "normal_x"
    -    normal_y: "normal_x"
    -    normal_z: "normal_x"
    -    intensity: "semantic label id"  
    -    verticality: "verticality of the terrain: proportional to normal_z (i.e., proportional to n_z in the planar model within the cell)"
    -    steepness_risk: "the roughness of the terrain: calculated based on discontinuity with surrounding terrain"
    -    inclination_risk: "the degree of smoothed inclination considering the surrounding terrain -> the closer to a wall or inclined terrain, the higher the risk"
    -    collision_risk: "collision area estimated based on the max_step_height (maximum surmountable height) set by the trip algorithm"
    -    height_noise: "uncertainty in height estimation"
    -    normal_noise: "uncertainty in the risk term and normal vector estimation"
    -    confidence: "a measure of how much confident data has been obtained or a measure of convergence"
    ```
    
- pcl::PointCloud<>
    ```ruby
    namespace trip_types {
        struct PointTRIP {
            PCL_ADD_POINT4D;
            PCL_ADD_NORMAL4D;
            PCL_ADD_INTENSITY;
            float min_elevation;  
            float verticality;    
            float steepness_risk;
            float inclination_risk;
            float collision_risk;  
            float height_noise;  
            float normal_noise;
            float confidence;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        } EIGEN_ALIGN16;
    };

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

    using PtRaw = pcl::PointXYZI;
    using PtTRIP = trip_types::PointTRIP;
    using ColorPtTRIP = pcl::PointXYZRGBL;
    ```

## Semantic label information
- label ID: semantic label (meaning)
    ```
    0: void     (장애물, 사람, 빌딩, 그 외)
    1: grass    (잔디)
    2: tree     (나무)
    3: water    (물, 물웅덩이)
    4: sky      (하늘)
    5: asphalt  (도로, 인도)
    6: rubble   (자갈, 돌)
    ```