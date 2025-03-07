
import numpy as np
import rospkg
import torch
import sys
library_path = "/home/sgvrjw/ros_ws/hanwha_2/src/hanwha-incorporation/mppi_controller/include/mppi_ctrl"
if library_path not in sys.path:
    sys.path.append(library_path)
from utils import wrap_to_pi_torch

rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('mppi_controller')

class GPGridMap:
    def __init__(self, device = "cuda", dt = 0.1):
        self.dt = dt
        self.vehicle_model = None
        self.map = None
        self.elev_map = None
        self.normal_vector = None
        self.right_corner_x = None
        self.right_corner_y = None
        self.map_resolution = None
        self.map_info = None
        self.c_size = None
        self.r_size = None
        self.surface_normal_x_idx = None
        self.surface_normal_y_idx = None
        self.surface_normal_z_idx = None
        self.elevation_idx = None        
        self.torch_device  = device
        self.elevationMap_torch = None
        self.normal_vector_x_torch = None
        self.normal_vector_y_torch = None
        self.normal_vector_z_torch = None
        

    def set_map(self,map):
        self.map = map 
        self.map_info = map.info           

        self.elevation_idx = self.map.layers.index("min_elevation")
        
        self.c_size =torch.tensor(map.data[self.elevation_idx].layout.dim[0].size).to(device=self.torch_device)             
        self.r_size = torch.tensor(map.data[self.elevation_idx].layout.dim[1].size ).to(device=self.torch_device)  

        
        self.map_resolution = torch.tensor(self.map_info.resolution).to(device=self.torch_device)    
         
        self.right_corner_x = torch.tensor(self.map_info.pose.position.x + self.map_info.length_x/2).to(device=self.torch_device)          
        self.right_corner_y = torch.tensor(self.map_info.pose.position.y + self.map_info.length_y/2).to(device=self.torch_device)  
        
        
        self.surface_normal_x_idx = self.map.layers.index("normal_x")            
        self.surface_normal_y_idx = self.map.layers.index("normal_y")            
        self.surface_normal_z_idx = self.map.layers.index("normal_z")   
                
        self.map.data[self.elevation_idx].data = np.nan_to_num(self.map.data[self.elevation_idx].data, nan = 0.0)           
        self.normal_vector_x = np.nan_to_num(self.map.data[self.surface_normal_x_idx].data, nan = 0.0)   
        self.normal_vector_y = np.nan_to_num(self.map.data[self.surface_normal_y_idx].data, nan = 0.0)   
        self.normal_vector_z = np.nan_to_num(self.map.data[self.surface_normal_z_idx].data, nan = 1.0)   
        
        self.update_torch_map()


    def update_torch_map(self):                   
        self.elevationMap_torch    = torch.from_numpy(self.map.data[self.elevation_idx].data).to(device=self.torch_device)            
        self.normal_vector_x_torch = torch.from_numpy(self.normal_vector_x).to(device=self.torch_device)            
        self.normal_vector_y_torch = torch.from_numpy(self.normal_vector_y).to(device=self.torch_device)            
        self.normal_vector_z_torch = torch.from_numpy(self.normal_vector_z).to(device=self.torch_device)            
        

    def normalize_vector(self,vector):        
        return vector / torch.sqrt(torch.sum(vector**2,1)).view(-1,1).repeat(1,3)     

    def get_normal_vector_idx(self,idx):
        assert self.surface_normal_x_idx is not None, "surface normal x is not available"        
        if torch.sum(idx >= self.r_size*self.c_size) + torch.sum(idx < 0) > 0:      
            default_normal_vector = torch.zeros(len(idx),3)    
            default_normal_vector[:,2] = 1.0
            return default_normal_vector           
        else:
            try:
                normal_vector = torch.vstack((torch.index_select(self.normal_vector_x_torch,0,idx.squeeze()), 
                                            torch.index_select(self.normal_vector_y_torch,0,idx.squeeze()),
                                                torch.index_select(self.normal_vector_z_torch,0,idx.squeeze())))          
                normal_vector = torch.transpose(normal_vector,0,1)            
                return self.normalize_vector(normal_vector)
            except:
                normal_vector = torch.zeros((len(idx), 3), device=self.torch_device)
                normal_vector[:,2] = 1.0
                return normal_vector

    def get_normal_vector(self,pose):
        if self.map is None or self.surface_normal_x_idx is None:
            return torch.Tensor([0,0,1])
        idx = self.pose2idx(pose)
        return self.get_normal_vector_idx(idx)                
    
    
    def get_elevation_idx(self,idx):
        if self.elevation_idx is None:
            return torch.zeros(len(idx))
        if torch.sum(idx >= self.r_size*self.c_size) > 0:
            print("idx out of bound")            
            return torch.zeros(len(idx))
        else: 
            return torch.index_select(self.elevationMap_torch,0,idx.squeeze())

            
    
    def get_elevation(self,pose):
        idx = self.pose2idx(pose)
        if torch.sum(idx < 0) > 0:
            return torch.zeros(len(idx))
            
        return self.get_elevation_idx(idx)        

  
    def get_rollpitch(self,pose):
        if not torch.is_tensor(pose):
            pose = torch.tensor(pose)  

        idx =self.pose2idx(pose)
        yaw = pose[:,2]
        default_rpy = torch.zeros(len(pose),3).to(device=self.torch_device) 
        default_rpy[:,2] = yaw
        if self.map is None:
            return default_rpy

        if torch.sum(idx) < 0:
            return default_rpy
        if self.surface_normal_x_idx is None:
            return default_rpy    
        normal_vector = self.get_normal_vector_idx(idx)             
        yaw_vec = torch.hstack([torch.cos(yaw).view(-1,1),torch.sin(yaw).view(-1,1),torch.zeros(len(yaw)).view(-1,1).to(device=self.torch_device)])
        yaw_vec[:,2] = -1*(normal_vector[:,0]*yaw_vec[:,0]+normal_vector[:,1]*yaw_vec[:,1])/(normal_vector[:,2]+1e-10)
        yaw_vec = self.normalize_vector(yaw_vec)        
       
        ry = torch.asin(yaw_vec[:,2])
        rz = torch.acos(yaw_vec[:,0] / ( torch.cos(ry)+1e-5))
        rx = torch.acos(normal_vector[:,2]/ (torch.cos(ry)+1e-5))
        roll = -1*rx
        pitch = -1*ry 
        yaw = rz 
        roll = wrap_to_pi_torch(roll)
        pitch = wrap_to_pi_torch(pitch)
               
        return torch.hstack([roll.view(-1,1), pitch.view(-1,1), yaw.view(-1,1)])
        
        
    def pose2idx(self,pose):  
        grid_c_idx = ((self.right_corner_x - pose[:,0].view(-1,1)) / self.map_resolution).int()
        grid_r_idx = ((self.right_corner_y - pose[:,1].view(-1,1)) / self.map_resolution).int()
        if torch.sum(grid_c_idx >= self.c_size) > 0:
            return -1*torch.ones(len(grid_c_idx)).int().to(device=self.torch_device) 
        if torch.sum(grid_r_idx >= self.r_size) > 0:            
            return -1*torch.ones(len(grid_c_idx)).int().to(device=self.torch_device)         
        idx = grid_c_idx + grid_r_idx*self.r_size 
        if torch.sum(idx >= self.c_size*self.r_size) + torch.sum(idx < 0) > 0:
            return -1*torch.ones(len(grid_c_idx)).int().to(device=self.torch_device)                  
        return idx.int()


    def idx2grid(self,idx):
        r_idx  = int(idx/self.c_size)
        c_idx  = idx%self.c_size
        return [r_idx, c_idx]


    
    