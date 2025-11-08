import numpy as np
import cv2
import sys


class Camera:
    def __init__(self, pipeline, config, align):
        self.pipeline = pipeline
        self.config = config
        self.align = align
        self.T_calib = None

    def get_aligned_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        return aligned_frames

    def get_img_and_depth(self):
        self.pipeline.start(self.config)
        frame_depth = np.asanyarray(self.get_aligned_frames().get_depth_frame().get_data())*0.25  # depth scale: 4X
        frame_color = np.asanyarray(self.get_aligned_frames().get_color_frame().get_data())
        self.pipeline.stop()
        return self.resize(frame_color), self.resize(frame_depth)
        
    def resize(self, img):
        """
        Crop and resize for the UOIS Input Size
        """
        return cv2.resize(img[:, 120:840], dsize=(640, 480), interpolation=cv2.INTER_AREA)

    def get_intrinsic_matrix(self):
        """
        Using modified intrinsics because of image cropping and resizing
        """
        self.pipeline.start(self.config)
        cam_intr = self.get_aligned_frames().get_color_frame().profile.as_video_stream_profile().intrinsics
        cam_matrix = np.asmatrix([[cam_intr.fx*8/9, 0, cam_intr.ppx*8/9-120*8/9],[0, cam_intr.fy*8/9, cam_intr.ppy*8/9],[0, 0, 1]])
        cam_distortion = cam_intr.coeffs
        self.pipeline.stop()
        return cam_matrix, tuple(cam_distortion)
    
    def set_calibration_matrix(self):
        CHECKERBOARD = (4,5)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        x = np.repeat([-2, -1, 0, 1, 2], 4)
        y = np.tile([-1, 0, 1, 2], 5)
        z = np.zeros(20)
        world_points = np.vstack([x, y, z]).T * 0.03
        imgpoints = []
        img, _ = self.get_img_and_depth()
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray,
                                                 CHECKERBOARD,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret == True:
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            imgpoints.append(corners2)
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            for i in range(0,len(corners2)):
                img=cv2.putText(
                    img, str(i), (int(corners2[i,0,0]),int(corners2[i,0,1])),cv2.FONT_HERSHEY_SIMPLEX, 1,(0,255,255),1,2)
            cv2.imshow('img',img)
            cv2.waitKey(3000)
        cv2.destroyAllWindows()
                
        cam_mtx, cam_dist = self.get_intrinsic_matrix()
        ret, rvec, tvec = cv2.solvePnP(world_points, imgpoints[0], cam_mtx, cam_dist)
        rmat, _ = cv2.Rodrigues(rvec)
        pose_world_cam = np.linalg.inv(np.r_[np.concatenate((rmat, tvec),1), [[0,0,0,1]]])
        self.T_calib = pose_world_cam