import pyrealsense2 as rs
import os
import numpy as np
from typing import *
from time import time
import open3d as o3d
import copy
import sys

from .cam import *
from .pcd_utils import *

# source from UOIS

import perception.src.data_augmentation as data_augmentation
import perception.src.segmentation as segmentation
import perception.src.util.utilities as util_


os.environ['CUDA_VISIBLE_DEVICES'] = "0"

class Perception:
    def __init__(self):
        context = rs.context()
        self.serials = [context.devices[i].get_info(
            rs.camera_info.serial_number) for i in range(2)]
        
        self.cams: List[Camera] = []
        for i in range(2):
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(self.serials[i])
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
            align_to = rs.stream.color
            align = rs.align(align_to)
            camera = Camera(pipeline, config, align)
            self.cams.append(camera)
    
    def calibrate(self):
        i = 1
        for cam in self.cams:
            cam.set_calibration_matrix()
            print("Camera ", i, " Calibration Done")
            i+=1

    def set_config(self):
        self.dsn_config = {
            # Sizes
            'feature_dim' : 64, # 32 would be normal

            # Mean Shift parameters (for 3D voting)
            'max_GMS_iters' : 10, 
            'epsilon' : 0.05, # Connected Components parameter
            'sigma' : 0.02, # Gaussian bandwidth parameter
            'num_seeds' : 200, # Used for MeanShift, but not BlurringMeanShift
            'subsample_factor' : 5,
            
            # Misc
            'min_pixels_thresh' : 500,
            'tau' : 15.,
        }
        self.rrn_config = {
            # Sizes
            'feature_dim' : 64, # 32 would be normal
            'img_H' : 224,
            'img_W' : 224,

            # architecture parameters
            'use_coordconv' : False,
        }
        self.uois3d_config = {
            # Padding for RGB Refinement Network
            'padding_percentage' : 0.25,
            
            # Open/Close Morphology for IMP (Initial Mask Processing) module
            'use_open_close_morphology' : True,
            'open_close_morphology_ksize' : 9,
            
            # Largest Connected Component for IMP module
            'use_largest_connected_component' : True,
        }
        checkpoint_dir = '/home/irsl/perception/perception/checkpoints/' # TODO: change this to directory of downloaded models
        self.dsn_filename = checkpoint_dir + 'DepthSeedingNetwork_3D_TOD_checkpoint.pth'
        self.rrn_filename = checkpoint_dir + 'RRN_OID_checkpoint.pth'
        self.uois3d_config['final_close_morphology'] = 'TableTop_v5' in self.rrn_filename

    def get_image_and_pcd(self, cam:Camera):
        cam_mtx, _ = cam.get_intrinsic_matrix()
        color, depth = cam.get_img_and_depth()

        z= depth.astype(float)/1000
        height, width = depth.shape
        px, py = np.meshgrid(np.arange(width), np.arange(height))
        px, py = px.astype(float), py.astype(float)
        x = ((px - cam_mtx[0, 2]) / cam_mtx[0, 0]) * z
        y = ((py - cam_mtx[1, 2]) / cam_mtx[1, 1]) * z
        pmap = np.concatenate([i[..., np.newaxis] for i in (x, y, z)], axis=-1)
        pmap_seg = pmap/10
        return color, pmap, pmap_seg        
   
    def get_object_mask(self, color:np.ndarray, pmap_seg:np.ndarray):
        """
        wrt the camera frame
        Segmentation using UOIS Model
        """

        rgb_imgs = np.zeros((1, 480, 640, 3), dtype=np.float32)
        xyz_imgs = np.zeros((1, 480, 640, 3), dtype=np.float32)

        # RGB
        rgb_img = np.zeros_like(color)
        rgb_img[:,:,0], rgb_img [:,:,1], rgb_img [:,:,2] = color[:,:,0], color[:,:,1], color[:,:,2]
        rgb_imgs[0] = data_augmentation.standardize_image(rgb_img)

        # XYZ
        xyz_imgs[0] = pmap_seg

        batch = {
            'rgb' : data_augmentation.array_to_tensor(rgb_imgs),
            'xyz' : data_augmentation.array_to_tensor(xyz_imgs),
        }

        ### Compute segmentation masks ###
        self.set_config()
        uois_net_3d = segmentation.UOISNet3D(
            self.uois3d_config, 
            self.dsn_filename, 
            self.dsn_config, 
            self.rrn_filename, 
            self.rrn_config)
        fg_masks, center_offsets, initial_masks, seg_masks = uois_net_3d.run_on_batch(batch)

        # Get results in numpy
        seg_masks = seg_masks.cpu().numpy()
        fg_masks = fg_masks.cpu().numpy()
        center_offsets = center_offsets.cpu().numpy().transpose(0,2,3,1)
        initial_masks = initial_masks.cpu().numpy()

        rgb_imgs = util_.torch_to_numpy(batch['rgb'].cpu(), is_standardized_image=True)
        num_objs = np.unique(seg_masks[0,...]).max() + 1
        
        rgb = rgb_imgs[0].astype(np.uint8)
        depth = xyz_imgs[0,...,2]
        seg_mask_plot = util_.get_color_mask(seg_masks[0,...], nc=num_objs)
        
        images = [rgb, depth, seg_mask_plot]
        titles = [f'Image {1}', 'Depth',
                f"Refined Masks. #objects: {np.unique(seg_masks[0,...]).shape[0]-1}"
                ]
        temp_mask = seg_mask_plot[:,:,0]+seg_mask_plot[:,:,1]+seg_mask_plot[:,:,2]
        obj_mask = np.where(temp_mask==0, 0, 1)

        plot_information = (images, titles)
        return obj_mask, plot_information
    
    def get_object_pcd_by_cam(self, cam:Camera, pose_world_cam_tuned):
        color, pmap, pmap_seg = self.get_image_and_pcd(cam)
        obj_mask, plot_information = self.get_object_mask(color, pmap_seg)
        obj_xyz = np.zeros((480, 640, 3), dtype=np.float32)
        obj_xyz[:,:,0], obj_xyz[:,:,1], obj_xyz[:,:,2] = pmap[:,:,0]*obj_mask, pmap[:,:,1]*obj_mask, pmap[:,:,2]*obj_mask
        pcd_cam = obj_xyz.reshape(-1,3)

        pcd_cam_obj_a = []
        for k in range(pcd_cam.shape[0]):
            if np.sum(pcd_cam[k]) != 0:
                pcd_cam_obj_a.append(pcd_cam[k])
        pcd_cam_obj = np.array(pcd_cam_obj_a)
        bar = np.ones((pcd_cam_obj.shape[0], 1), dtype=np.float32)
        pcd_temp = np.concatenate((pcd_cam_obj,bar),1)
        pcd_world = np.rot90(np.dot(pose_world_cam_tuned, np.rot90(pcd_temp,3)), 1)[:,0:3]
        return pcd_world, plot_information
    
    def get_object_pcds(self, tvec_tuning_const):
        pcds = []
        plot_informations = []
        for i, cam in enumerate(self.cams):
            trans_tuning = np.r_[np.concatenate((np.identity(3), tvec_tuning_const[i]),1), [[0,0,0,1]]]
            pose_world_cam_tuned = np.matmul(trans_tuning, cam.T_calib)
            pcd, plot_information = self.get_object_pcd_by_cam(cam, pose_world_cam_tuned)
            pcds.append(pcd)
            plot_informations.append(plot_information)
        return pcds, plot_informations # a list of pcds
    
    def get_input_for_icp(self, obj_name, pcds, voxel_size, sample_point_number=1000):
        print(":: Load two point clouds and disturb initial pose.")
        source_raw = o3d.geometry.PointCloud()
        source_raw.points = o3d.utility.Vector3dVector(np.vstack(pcds))
        source_inlier = remove_outlier(source_raw)
        source_all = get_pcd_with_base(source_inlier)
        source = farthest_point_sampling(source_all, sample_point_number)
        source.paint_uniform_color([0.5, 0., 0])
    
        target_all = o3d.io.read_point_cloud("/home/irsl/perception/perception/gt_objects/"+obj_name+"_gt.ply")
        target = farthest_point_sampling(target_all, sample_point_number)
        print(target)
        trans_init = np.eye(4)
        source.transform(trans_init)
        print(source)
        self.draw_registration_result(source, target, np.identity(4))
        source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

        return source, target, source_down, target_down, source_fpfh, target_fpfh

    def get_T_object_wrt_world(self, pcds, obj_name, est_const):
        voxel_size = 0.2
        source, target, source_down, target_down, source_fpfh, target_fpfh = self.get_input_for_icp(
                                                            obj_name, pcds, voxel_size, sample_point_number=1000)
        while True:
            result_ransac = execute_global_registration(source_down, target_down,
                                                        source_fpfh, target_fpfh,
                                                        voxel_size)
            result_icp = refine_registration(source, target, result_ransac, voxel_size)
            if result_icp.fitness > est_const[0] and result_icp.inlier_rmse < est_const[1]:
                print(result_icp)
                self.draw_registration_result(source, target, result_icp.transformation)
                break

        return result_icp.transformation

    ## visualization
    def visualize_scene(self, pcds, frame_size=0.1):
        colors = [[0,0,1], [1,0,0]]
        pcd_o3ds = []
        frames = []
        for i in range(2):
            pcd_o3d_raw= o3d.geometry.PointCloud()
            pcd_o3d_raw.points = o3d.utility.Vector3dVector(pcds[i])
            pcd_o3d_no_base = remove_outlier(pcd_o3d_raw)
            pcd_o3d = get_pcd_with_base(pcd_o3d_no_base)
            pcd_o3d.paint_uniform_color(colors[i])             
            pcd_o3ds.append(pcd_o3d)
            
            coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)
            coord.transform(self.cams[i].T_calib)
            frames.append(coord)
        coord_world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)
        frames.append(coord_world)
        return pcd_o3ds, frames
    
    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.03)
        o3d.visualization.draw_geometries([source_temp, target_temp, coord])