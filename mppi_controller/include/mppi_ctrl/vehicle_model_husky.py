import torch
import sys
library_path = "/home/sgvrjw/ros_ws/hanwha_2/src/hanwha-incorporation/mppi_controller/include/mppi_ctrl"
if library_path not in sys.path:
    sys.path.append(library_path)
from utils import b_to_g_rot
from mppi import MPPI
import tf2_ros
import tf2_geometry_msgs
import rospy

class VehicleModel:
    def __init__(self, device_ = "cuda", dt = 0.1, N_node = 10, local_map = None, mppi_n_sample = 1000):
        self.local_map = local_map
        self.N = N_node
        self.g= 9.814195
        self.h = 0.15
        self.dt = dt 
        self.rollover_cost_scale = 0.2     
        self.traj_following_cost_scale =15.0 #30.0
        self.mppi_n_sample = mppi_n_sample
        self.torch_device = device_
        self.state_dim = 6
        self.input_dim = 2
        self.lambda_ = 0.1
        self.horizon = self.dt* N_node
        self.noise_sigma = torch.diag(torch.ones(self.input_dim) * 1, 0)
        
        # lin_vel and ang_vel
        self.u_min = torch.tensor([0.0, -0.2]).to(device= self.torch_device)
        self.u_max = torch.tensor([1.5, 0.2]).to(device= self.torch_device)

        self.mppi = MPPI(self.dynamics_update, self.running_cost, self.state_dim, self.noise_sigma, num_samples=self.mppi_n_sample, horizon=self.N,
                         lambda_=self.lambda_, u_min = self.u_min, u_max = self.u_max)
        
        self.target_goal = None # local waypoint from planned path


    def running_cost(self, state, action, t, prev_state):
        # self.target_goal --> [x, y]
        cost = torch.zeros(state.shape[0]).to(device=self.torch_device)

        ############################################################
        ################# Trajectory following cost ################
        ############################################################

        goal_dist_cost = torch.zeros(state.shape[0]).to(device=self.torch_device)
        
        if t == self.N // 3:
            goal_dist_cost = torch.norm(state[:,0:2] - self.target_goal[0, 0:2], dim = 1) * 0.5
            
        elif t == 2 * self.N // 3:
            goal_dist_cost = torch.norm(state[:,0:2] - self.target_goal[1, 0:2], dim = 1) * 0.3
        
        elif t == self.N-1:
            goal_dist_cost = torch.norm(state[:,0:2] - self.target_goal[2, 0:2], dim = 1) * 0.2
            
        # if t == self.N-1:
        #     goal_dist_cost = torch.norm(state[:,0:2] - self.target_goal[2, 0:2], dim = 1)
        
        # goal_dist_cost_1 = torch.norm(state[:,0:2] - self.target_goal[0, 0:2], dim = 1) # n_samples x 1
        # goal_dist_cost_2 = torch.norm(state[:,0:2] - self.target_goal[1, 0:2], dim = 1) # n_samples x 1
        # goal_dist_cost_3 = torch.norm(state[:,0:2] - self.target_goal[2, 0:2], dim = 1) # n_samples x 1
        # goal_dist_costs = torch.stack((goal_dist_cost_1, goal_dist_cost_2, goal_dist_cost_3), dim=1)  # n_samples x 3
        
        # goal_dist_cost = torch.min(goal_dist_costs, dim=1)[0] # n_samples x 1
        
        roll_threshold = 0.8
        pitch_threshold = 1.0
        
        roll = state[:,4]
        pitch = state[:,5]
        
       # rollover_cost = torch.tanh((roll / roll_threshold)) + torch.tanh((pitch / pitch_threshold))
        
        cost = self.traj_following_cost_scale * goal_dist_cost# + self.rollover_cost_scale * rollover_cost

        return cost
    
    def set_target_goal(self, goal):
        if goal is not None:
            self.target_goal = torch.tensor(goal, device="cuda") # [x, y, z] # base link frame
        else:
            self.target_goal = torch.zeros((3, 3), device="cuda") # [x, y, z] # base link frame

    
    def dynamics_update(self,x,u):     
        # x(0), y(1), yaw(2), z(3) roll(4) pitch(5) # all in base_link frame # when t == 0, all values are zero                
        # x is N x 6, u is N x 2

        # u(0) = lin_v, u(1) = angular_v  # all in base frame
        
        if not torch.is_tensor(x):
            x = torch.tensor(x).to(device=self.torch_device)    
        if not torch.is_tensor(u):
            u = torch.tensor(u).to(device=self.torch_device)    
        nx = torch.clone(x).to(device=self.torch_device)
        
        
        ######## get roll and pitch from Map ######## 
        if self.local_map.elevationMap_torch is not None:            
            pose = torch.transpose(torch.vstack([x[:,0],x[:,1],x[:,2]]), 0, 1) # N x 3 in base_link frame (x, y, yaw)
    
            rpy_tmp = self.local_map.get_rollpitch(pose)  # change pose to idx, get normal vector, and calculate roll, pitch, yaw
            roll = rpy_tmp[:,0]
            pitch = rpy_tmp[:,1]
            y_ = rpy_tmp[:,2]
            x[:,3] = self.local_map.get_elevation(pose)
            nx[:,3] = x[:,3]
            
        else:        
            roll = x[:,4]
            pitch = x[:,5]
            y_ = x[:,2]

        # extract linear and angular velocities from control input
        lin_vel = u[:,0]
        ang_vel = u[:,1]
        
        # calculate 3D rot matrix from base to world
        # rot_base_to_world = b_to_g_rot(roll,pitch,x[:, 2]).double()    
        # local_vel = torch.hstack([lin_vel.view(-1,1),torch.zeros_like(lin_vel, device="cuda").view(-1,1),torch.zeros(len(x[:,3])).to(device=self.torch_device).view(-1,1)]).view(-1,3,1).double()        
        # vel_in_world = torch.bmm(rot_base_to_world, local_vel).view(-1,3)
        
        # vxw = vel_in_world[:,0]
        # vyw = vel_in_world[:,1]
        # vzw = vel_in_world[:,2]   
                    
        nx[:,0] = nx[:,0]+self.dt*lin_vel*torch.cos(x[:, 2])
        nx[:,1] = nx[:,1]+self.dt*lin_vel*torch.sin(x[:, 2])
        nx[:,2] = nx[:,2]+self.dt*ang_vel
        
        next_pose = torch.transpose(torch.vstack([nx[:,0],nx[:,1],nx[:,2]]), 0, 1)
        nx[:, 3] = self.local_map.get_elevation(next_pose)
        # nx[:,3] = nx[:,3]+self.dt*vzw
        
        if self.local_map.elevationMap_torch is not None:
            # next_pose_map_frame = torch.transpose(torch.vstack([nx[:,0],nx[:,1],nx[:,2]]), 0, 1)  # N x 3
            # next_pose_base_frame = self.transform_pose_to_base_link(next_pose_map_frame, transform)
            rpy_next = self.local_map.get_rollpitch(next_pose)  # change pose to idx, get normal vector, and calculate roll, pitch, yaw
            nx[:,4] = rpy_next[:,0]
            nx[:,5] = rpy_next[:,1]
        else:
            nx[:,4] = roll
            nx[:,5] = pitch
            
        # next_pose = torch.cat((nx[:, 0].unsqueeze(-1), nx[:, 1].unsqueeze(-1), nx[:, 3].unsqueeze(-1)), dim=-1)

        next_pose_xyz = torch.transpose(torch.vstack([nx[:,0],nx[:,1],nx[:,3]]), 0, 1)

        return nx, next_pose_xyz