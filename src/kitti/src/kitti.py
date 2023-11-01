#!/usr/bin/env python3
import os
from cv_bridge import CvBridge
from data_utils import *
from publish_utils import *
from misc import *
import rospy

DATA_PATH = './a_data/'
EGOCAR = np.array([[2.15,0.9,-1.73],[2.15,-0.9,-1.73],[-1.95,-0.9,-1.73],[-1.95,0.9,-1.73],
                    [2.15,0.9,-0.23],[2.15,-0.9,-0.23],[-1.95,-0.9,-0.23],[-1.95,0.9,-0.23]])

if __name__ == '__main__':
    frame = 0
    rospy.init_node('talker',anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam',Image,queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud',PointCloud2,queue_size=10)
    ego_pub = rospy.Publisher('kitti_ego_car',MarkerArray,queue_size=10)
    imu_pub = rospy.Publisher('kitti_imu',Imu,queue_size=10)
    gps_pub = rospy.Publisher('kitti_gps',NavSatFix,queue_size=10)
    box3d_pub = rospy.Publisher('kitti_3d',MarkerArray,queue_size=10)
    loc_pub = rospy.Publisher('kitti_loc',MarkerArray,queue_size=10)
    dist_pub = rospy.Publisher('kitti_dist',MarkerArray,queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(10)

    df_tracking = read_tracking(os.path.join(DATA_PATH,'tracking/training/label_02/0000.txt'))
    calib = Calibration(os.path.join(DATA_PATH,'RawData/2011_09_26/'),from_video=True)

    # ego_car = Object()
    tracker = {} #track_id : Object
    prev_imu_data = None

    while not rospy.is_shutdown():
        # each frame boxes
        boxes_2d = np.array(df_tracking[df_tracking.frame==frame][['bbox_left','bbox_top','bbox_right','bbox_bottom']])
        # diffrient colar of object in each frame
        types = np.array(df_tracking[df_tracking.frame==frame]['type'])
        boxes_3d = np.array(df_tracking[df_tracking.frame==frame][['height','width','length','pos_x','pos_y','pos_z','rot_y']])
        track_ids = np.array(df_tracking[df_tracking.frame==frame]['track_id'])

        corners_3d_velos = []
        centers = {} #track_id :center
        minPQDs = [] #P,Q for shortest points, D for shortest distance

        for track_id,box_3d in zip(track_ids,boxes_3d):
            corners_3d_cam2 = compute_3d_box_cam2(*box_3d)
            corners_3d_velo = calib.project_rect_to_velo(corners_3d_cam2.T)
            minPQDs += [min_distance_cuboids(EGOCAR,corners_3d_velo)]
            corners_3d_velos += [corners_3d_velo]
            centers[track_id] = np.mean(corners_3d_velo,axis = 0)[:2]
        corners_3d_velos += [EGOCAR]
        types = np.append(types,'Car')
        track_ids = np.append(track_ids,-1)
        centers[-1] = np.array([0,0])

        image = read_camera(os.path.join(DATA_PATH,'RawData/2011_09_26/2011_09_26_drive_0005_sync/image_02/data/%010d.png'%frame))
        point_cloud = read_point_cloud(os.path.join(DATA_PATH,'RawData/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/%010d.bin'%frame))   
        imu_data = read_imu(os.path.join(DATA_PATH,'RawData/2011_09_26/2011_09_26_drive_0005_sync/oxts/data/%010d.txt'%frame))

        if prev_imu_data is None:
            for track_id in centers:
                tracker[track_id] = Object(centers[track_id])
        else:
            displacement = 0.1 * np.linalg.norm(imu_data[['vf', 'vl']])
            yaw_change = float(imu_data.yaw.iloc[0]- prev_imu_data.yaw.iloc[0])
            for track_id in centers: #for each object in present frame
                if track_id in tracker:
                    tracker[track_id].update(centers[track_id],displacement,yaw_change)
                else:
                    tracker[track_id] = Object(centers[track_id])
            for track_id in tracker:
                if track_id not in centers:
                    tracker[track_id].update(None,displacement,yaw_change)

        prev_imu_data = imu_data

        publish_camera(cam_pub,bridge,image,boxes_2d,types)   
        publish_point_cloud(pcl_pub,point_cloud)
        publish_ego_car(ego_pub)
        publish_imu(imu_pub,imu_data)
        publish_gps(gps_pub,imu_data)
        publish_3dbox(box3d_pub,corners_3d_velos,types,track_ids)
        publish_loc(loc_pub,tracker,centers)
        publish_dist(dist_pub,minPQDs)
        rospy.loginfo("published frame  : %d"%frame )
        rate.sleep()
        
        frame +=1
        if frame == 154:
            frame = 0
            for track_id in tracker:
                tracker[track_id].reset()
        

