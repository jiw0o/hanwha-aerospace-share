#!/usr/bin/env python3.8
import sys, os
import rospy
import time
import numpy as np
import math 
from std_msgs.msg import Bool, ColorRGBA, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import MarkerArray, Marker
from grid_map_msgs.msg import GridMap
import torch
import tf
# import tf2_ros
# import tf2_geometry_msgs
import rospkg
rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('mppi_controller')
library_path = pkg_dir + "/include/mppi_ctrl"
if library_path not in sys.path:
    sys.path.append(library_path)
from vehicle_model_husky import VehicleModel
from utils import get_odom_euler, wrap_to_pi
from gpgridmap import GPGridMap
from planning_module.msg import PathData


class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.data = []

    def apply(self, value):
        """Add a new value and compute the moving average."""
        self.data.append(value)
        if len(self.data) > self.window_size:
            self.data.pop(0)
        return sum(self.data) / len(self.data)





class MPPIWarpper:
    def __init__(self):       

        self.prediction_hz = rospy.get_param('~prediction_hz', default=50)
        self.n_nodes = rospy.get_param('~n_nodes', default=10)
        self.t_horizon = rospy.get_param('~t_horizon', default=10.0) #6.0                  
        self.mppi_n_sample = rospy.get_param('~mppi_n_sample', default=500)                           
        self.torch_device = "cuda:0"   ## Specify the name of GPU 
        self.torch_dtype  = torch.double
        self.dt = self.t_horizon / self.n_nodes * 1.0        
        # x, y, yaw, z, roll, pitch 
        # 0  1   2   3   4       5  \
        self.cur_x = np.transpose(np.zeros([1, 6]))        
        self.path_look_ahead_idx = 1
        # Initialize Grid map instance and 3D vehicle model         
        self.local_map = GPGridMap(device = self.torch_device, dt = self.dt)
        self.VehicleModel = VehicleModel(device_ = self.torch_device, dt = self.dt,N_node = self.n_nodes,  local_map = self.local_map, mppi_n_sample = self.mppi_n_sample)        
        self.local_map.vehicle_model = self.VehicleModel
        
        self.goal_pose_list_w = None
        self.goal_pose_list_b = None
        
        self.planned_path = None
        self.planned_odom = None
        
        self.first_path_flag = False

        self.odom_available   = False 
        self.map_available = False
             
        self.odom = Odometry()
        self.waypoint = PoseStamped()        
        
        # Subscriber Topics
        # odom_topic = "/ground_truth/state"             
        odom_topic = "/odometry_gt"
        local_map_topic = "/trip/trip_updated/terrain_local_gridmap" #"/traversability_estimation/global_map"        
        path_topic = "/path_with_odom"        
        
        # Publisher Topics
        control_topic = "/husky_velocity_controller/cmd_vel"                        
        pred_traj_topic = "/mppi_pred_path"
        local_goal_topic = "/local_goal"
        
        mppi_status_topic = "/mppi_result"
        
        self.action = None
                
        # Subscribers
        self.goal_sub = rospy.Subscriber(path_topic, PathData, self.goal_callback) 
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback) 
        self.local_map_sub = rospy.Subscriber(local_map_topic, GridMap, self.gridmap_callback) 
        
        # Publishers                        
        self.control_pub = rospy.Publisher(control_topic, Twist, queue_size=1, tcp_nodelay=True) # husky_velocity_controller/cmd_vel
        self.pred_traj_pub = rospy.Publisher(pred_traj_topic, MarkerArray, queue_size=1)
        # self.local_goal_pub = rospy.Publisher(local_goal_topic, Marker, queue_size=1)
        self.local_goal_pub = rospy.Publisher(local_goal_topic, MarkerArray, queue_size=1)
        
        self.mppi_status_pub = rospy.Publisher(mppi_status_topic, Int32, queue_size=1)

        # controller callback         
        self.cmd_timer = rospy.Timer(rospy.Duration(1/self.prediction_hz), self.cmd_callback)   
        self.command_pub = rospy.Timer(rospy.Duration(1/self.prediction_hz), self.cmd_pub)
        self.local_goal_timer = rospy.Timer(rospy.Duration(1/10.0), self.local_goal_callback)
        
        self.linear_filter = MovingAverageFilter(window_size=5)
        self.angular_filter = MovingAverageFilter(window_size=5)
    
              
        rate = rospy.Rate(1)     
        while not rospy.is_shutdown():            
            msg = Bool()
            msg.data = True
            rate.sleep()
            
    def euler_from_quaternion(self, quaternion):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        Quaternion is a tuple/list (x, y, z, w)
        Returns: tuple (roll, pitch, yaw) in radians
        """
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler 

    def get_odom_euler(self, odom_msg):
        # Assuming this function converts quaternion to Euler angles and returns (roll, pitch, yaw)
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = self.euler_from_quaternion(orientation_list)  
        return (roll, pitch, yaw)
    
    def calculate_rotation_matrix_from_path_odom_to_robot(self, path_odom_msg, robot_x, robot_y, robot_yaw):
        # Get the yaw of the path_odom frame
        path_odom_euler = self.get_odom_euler(path_odom_msg)
        path_odom_yaw = path_odom_euler[2]  # Yaw of path_odom in the world frame

        # Compute the relative yaw between robot and path_odom
        relative_yaw = path_odom_yaw - robot_yaw  # This gives the yaw difference (robot frame vs. path_odom frame)

        # Now, construct the rotation matrix to transform from path_odom to the robot's frame
        cos_yaw = math.cos(relative_yaw)
        sin_yaw = math.sin(relative_yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])

        return rotation_matrix


    def goal_callback(self, msg):
        self.planned_odom = msg.path_odom
        self.planned_path = msg.path_nodes.markers #[8:]
        
        self.first_path_flag = True
        

    def local_goal_callback(self, timer):
        if self.planned_path is None:
            return
        
        robot_x = self.odom.pose.pose.position.x
        robot_y = self.odom.pose.pose.position.y
        current_euler = get_odom_euler(self.odom)
        robot_yaw = current_euler[2]
        
        waypoint_dist = [3.0, 6.0, 9.0]
        
        yaw_rad = robot_yaw
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        
        path_odom = self.planned_odom
        rot_matrix = self.calculate_rotation_matrix_from_path_odom_to_robot(path_odom, robot_x, robot_y, robot_yaw) #odom frame to robot frame

        trans = [robot_x - path_odom.pose.pose.position.x, robot_y - path_odom.pose.pose.position.y]
        trans_x = trans[0]
        trans_y = trans[1]
        
        rotation_matrix_1 = np.array([[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]])
        
        goal_pose_list = []
        goal_pose_ori_list = []
        
        first_waypoint_index = 0
        
        
        last_marker = self.planned_path[-1]
        last_marker_x = last_marker.pose.position.x
        last_marker_y = last_marker.pose.position.y
        temp = np.array([last_marker_x, last_marker_y])
        last_marker_base = np.dot(rot_matrix, temp) - np.dot(rotation_matrix_1, np.array([trans_x, trans_y]))
        
        for i in range(len(waypoint_dist)):
            goal_pose = []
            goal_ori_pose = []
            closest_distance = float('inf')
            for j, marker in enumerate(self.planned_path):
            # for j in range(len(self.planned_path) - 1, -1, -1):
                # marker = self.planned_path[j]
                marker_x = marker.pose.position.x # path odom frame
                marker_y = marker.pose.position.y # path odom frame
                marker_z = marker.pose.position.z # path odom frame
                
                temp = np.array([marker_x, marker_y])
                marker_base = np.dot(rot_matrix, temp) - np.dot(rotation_matrix_1, np.array([trans_x, trans_y]))
                
                if marker_base[0] < 0.1:
                    continue

                distance = math.sqrt((marker_base[0])**2 + (marker_base[1])**2)
                
                # if distance < 1.0 and len(self.planned_path) > 3:
                #     del self.planned_path[j]
                #     continue
    
                differences = abs(distance - waypoint_dist[i])
                if differences < closest_distance:
                    closest_distance = differences
                    goal_pose = [marker_base[0], marker_base[1], marker_z] # base_link frame
                    goal_ori_pose = [marker_x, marker_y, marker_z] # paht odom frame
                    
                    if i == 0:
                        first_waypoint_index = j
            
            if len(goal_pose) == 0:
                goal_pose = [last_marker_base[0], last_marker_base[1], last_marker.pose.position.z]
                goal_ori_pose = [last_marker_x, last_marker_y, last_marker.pose.position.z]

            goal_pose_list.append(goal_pose)
            goal_pose_ori_list.append(goal_ori_pose)
        
        self.goal_pose_list_b = goal_pose_list # base_link frame
        
        # if self.first_path_flag:
        #     del self.planned_path[0:first_waypoint_index+1]
        #     self.first_path_flag = False
            
        
        marker_array = MarkerArray()
        for i in range(len(goal_pose_list)):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.pose.position.x = goal_pose_list[i][0]
            marker.pose.position.y = goal_pose_list[i][1]
            marker.pose.position.z = goal_pose_list[i][2]
            marker.pose.orientation.w = 1.0
            marker.lifetime = rospy.Duration(0.1)
            marker.scale.x = 0.7
            marker.scale.y = 0.7
            marker.scale.z = 0.7
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
            marker_array.markers.append(marker)
        self.local_goal_pub.publish(marker_array)
        
        
        goal_pose_w_list = []
        for i in range(len(goal_pose_list)):
            marker_position_base = np.array([goal_pose_list[i][0], goal_pose_list[i][1]]) # base_link frame
            marker_position_world = np.dot(rotation_matrix, marker_position_base) + np.array([robot_x, robot_y]) # world frame
            goal_pose_w_list.append([marker_position_world[0], marker_position_world[1], goal_pose_list[i][2]])
            
        self.goal_pose_list_w = goal_pose_w_list # world frame

        

    def gridmap_callback(self,msg):                     
        if self.map_available is False:
            self.map_available = True
        self.local_map.set_map(msg)


    def odom_callback(self, msg):
        """                
        :type msg: PoseStamped
        """              
        if self.odom_available is False:
            self.odom_available = True 
        self.odom = msg        

        
    def cmd_callback(self,timer):
    
        if self.odom_available is False:
            rospy.loginfo("Odom is not available yet")
            return
        elif self.map_available is False:
            rospy.loginfo("Map is not available yet")
            return        
        elif self.goal_pose_list_w is None:
            return
 
        current_euler = get_odom_euler(self.odom)

        for i in range(3):
            current_euler[i] = wrap_to_pi(current_euler[i])

        # x, y, yaw, z, roll, pitch 
        # 0  1   2   3   4       5  \
        self.cur_x_world = np.transpose(np.array([self.odom.pose.pose.position.x, # world frame
                                            self.odom.pose.pose.position.y, # world frame
                                            current_euler[2], # world frame
                                            self.odom.pose.pose.position.z, # world frame                              
                                            current_euler[0], # world frame
                                            current_euler[1]] # world frame
                                           ))

        self.cur_x = np.zeros_like(self.cur_x_world)

        x = torch.tensor(self.cur_x).to(dtype=self.torch_dtype, device=self.torch_device) # local frame # all values are zero

        robot_x = self.odom.pose.pose.position.x # world frame
        robot_y = self.odom.pose.pose.position.y # world frame
        robot_yaw = current_euler[2] # world frame
        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        current_goal_b = []
        for i in range(len(self.goal_pose_list_w)):
            goal_x = self.goal_pose_list_w[i][0]
            goal_y = self.goal_pose_list_w[i][1]
            relative_x = goal_x - robot_x
            relative_y = goal_y - robot_y
            goal_position_base = np.dot(rotation_matrix, np.array([relative_x, relative_y]))
            current_goal_b.append([goal_position_base[0], goal_position_base[1], self.goal_pose_list_w[i][2]])
        
        
        first_goal_yaw = math.atan2(current_goal_b[0][1], current_goal_b[0][0])
        
        if abs(first_goal_yaw) > 0.78: # 45 degree
            # print("staop and rotate")
            if first_goal_yaw > 0:
                self.action = [0.0, 0.3]
            else:
                self.action = [0.0, -0.3]
        
        else:
            self.VehicleModel.set_target_goal(current_goal_b) 

            start = time.time()
            action, min_index = self.VehicleModel.mppi.command(x) # linear velocity, angular velocity
            end = time.time()
            # print("mppi time: ", end-start)     
            mppi_pred_traj = MarkerArray()
            K, T, _ = self.VehicleModel.mppi.states[0].shape
            
            # for k in range(K):
            k = min_index
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.id = k
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.1)
            
            for t in range(T):
                point = Point()
                point.x = self.VehicleModel.mppi.predicted_traj[t][0, k, 0].item()
                point.y = self.VehicleModel.mppi.predicted_traj[t][0, k, 1].item()
                point.z = self.VehicleModel.mppi.predicted_traj[t][0, k, 2].item() + 1.7
                if k == min_index:
                    point.z += 0.2
                marker.points.append(point)
            
            marker.scale.x = 0.02
            if k == min_index:
                marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
                marker.scale.x = 0.2
            else:
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.7)
                marker.scale.x = 0.01
                
            mppi_pred_traj.markers.append(marker)

            self.pred_traj_pub.publish(mppi_pred_traj)
            
            # import ipdb; ipdb.set_trace()
            
            
            filtered_linear = self.linear_filter.apply(action[0].item())
            filtered_angular = self.angular_filter.apply(action[1].item())
            
            
            self.action = [filtered_linear, filtered_angular]
        
        
        status_msg = Int32()
        status_msg.data = 1
        self.mppi_status_pub.publish(status_msg)

    
    def cmd_pub(self, timer):
        if self.action is None:
            self.action = [0.0, 0.0]
        target_action = Twist()
        target_action.linear.x = self.action[0]
        target_action.angular.z = self.action[1]
        self.control_pub.publish(target_action)
        
        # print(self.action)
        
    
    
###################################################################################

def main():
    rospy.init_node("mppi_ctrl")    
    MPPIWarpper()

if __name__ == "__main__":
    main()