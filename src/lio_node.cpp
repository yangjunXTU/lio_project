/*
 * @Auther: yangj
 * @Description:  
 * @Date: 2025-06-19 09:17:49
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-10-29 08:22:40
 */

#pragma once
#include"utils/lio_node.h"
#include <stdexcept>
#include <cmath>
#include <filesystem>

namespace {

bool LoadVectorParam(ros::NodeHandle& nh, const std::string& key, std::vector<double>& values, size_t expected_size) {
    if (!nh.getParam(key, values)) {
        ROS_ERROR_STREAM("Missing parameter: " << key);
        return false;
    }
    if (values.size() != expected_size) {
        ROS_ERROR_STREAM("Parameter " << key << " must have " << expected_size << " values, got " << values.size());
        return false;
    }
    return true;
}

bool LoadVectorParam(ros::NodeHandle& nh, const std::string& key, std::vector<int>& values, size_t expected_size) {
    if (!nh.getParam(key, values)) {
        ROS_ERROR_STREAM("Missing parameter: " << key);
        return false;
    }
    if (values.size() != expected_size) {
        ROS_ERROR_STREAM("Parameter " << key << " must have " << expected_size << " values, got " << values.size());
        return false;
    }
    return true;
}

bool ValidateRotationMatrix(const Eigen::Matrix3d& R, const std::string& key) {
    const Eigen::Matrix3d should_be_identity = R.transpose() * R;
    const double orthogonality_error = (should_be_identity - Eigen::Matrix3d::Identity()).norm();
    const double determinant_error = std::abs(R.determinant() - 1.0);
    if (orthogonality_error > 1e-3 || determinant_error > 1e-3) {
        ROS_ERROR_STREAM("Invalid rotation matrix in " << key
                         << ", orthogonality error = " << orthogonality_error
                         << ", determinant error = " << determinant_error);
        return false;
    }
    return true;
}

bool LoadMatrix3dParam(ros::NodeHandle& nh, const std::string& key, Eigen::Matrix3d& R) {
    std::vector<double> values;
    if (!LoadVectorParam(nh, key, values, 9)) {
        return false;
    }
    R = MatFromArray(values);
    return ValidateRotationMatrix(R, key);
}

bool LoadVector3dParam(ros::NodeHandle& nh, const std::string& key, Eigen::Vector3d& t) {
    std::vector<double> values;
    if (!LoadVectorParam(nh, key, values, 3)) {
        return false;
    }
    t = VecFromArray(values);
    return true;
}

bool LoadCameraConfig(ros::NodeHandle& nh, CameraConfig& camera_config) {
    std::vector<int> image_size;
    std::vector<double> K_values;

    nh.param<std::string>("camera/model", camera_config.model, std::string("pinhole"));
    nh.param<double>("camera/scale", camera_config.scale, 1.0);

    if (!LoadVectorParam(nh, "camera/image_size", image_size, 2)) {
        return false;
    }
    if (!LoadVectorParam(nh, "camera/K", K_values, 9)) {
        return false;
    }
    if (!nh.getParam("camera/D", camera_config.D)) {
        ROS_ERROR_STREAM("Missing parameter: camera/D");
        return false;
    }
    if (camera_config.D.empty()) {
        ROS_ERROR_STREAM("Parameter camera/D must not be empty");
        return false;
    }

    camera_config.width = image_size[0];
    camera_config.height = image_size[1];
    camera_config.K = MatFromArray(K_values);
    return true;
}

bool LoadVioConfig(ros::NodeHandle& nh, VioConfig& vio_config) {
    nh.param<int>("vio/pyr_levels", vio_config.pyr_levels, 3);
    nh.param<int>("vio/half_patch_size", vio_config.half_patch_size, 1);
    nh.param<int>("vio/max_points", vio_config.max_points, 400);
    nh.param<int>("vio/gn_iters_per_level", vio_config.gn_iters_per_level, 5);
    nh.param<int>("vio/min_valid_residuals", vio_config.min_valid_residuals, 80);
    nh.param<int>("vio/grid_rows", vio_config.grid_rows, 6);
    nh.param<int>("vio/grid_cols", vio_config.grid_cols, 8);
    nh.param<int>("vio/max_points_per_cell", vio_config.max_points_per_cell, 2);
    nh.param<double>("vio/min_gradient", vio_config.min_gradient, 10.0);
    nh.param<double>("vio/min_depth", vio_config.min_depth, 1.0);
    nh.param<double>("vio/max_depth", vio_config.max_depth, 30.0);
    nh.param<double>("vio/huber_delta", vio_config.huber_delta, 10.0);
    nh.param<double>("vio/info_scale", vio_config.info_scale, 0.01);
    nh.param<double>("vio/min_inlier_ratio", vio_config.min_inlier_ratio, 0.6);
    nh.param<double>("vio/max_mean_residual", vio_config.max_mean_residual, 12.0);
    nh.param<double>("vio/max_update_translation", vio_config.max_update_translation, 0.3);
    nh.param<double>("vio/max_update_rotation_deg", vio_config.max_update_rotation_deg, 8.0);
    nh.param<double>("vio/min_keyframe_translation", vio_config.min_keyframe_translation, 0.15);
    nh.param<double>("vio/min_keyframe_rotation_deg", vio_config.min_keyframe_rotation_deg, 5.0);
    return true;
}

bool LoadSemanticConfig(ros::NodeHandle& nh, SemanticConfig& semantic_config) {
    nh.param<bool>("semantic/enabled", semantic_config.enabled, true);
    nh.param<bool>("semantic/publish_map_point_rgb", semantic_config.publish_map_point_rgb, true);
    nh.param<bool>("semantic/publish_map_semantic", semantic_config.publish_map_semantic, true);
    nh.param<bool>("semantic/highlight_vio_points", semantic_config.highlight_vio_points, true);
    nh.param<double>("semantic/min_project_depth", semantic_config.min_project_depth, 1.0);
    nh.param<double>("semantic/max_project_depth", semantic_config.max_project_depth, 30.0);
    nh.param<double>("semantic/color_fusion_alpha", semantic_config.color_fusion_alpha, 0.7);
    nh.param<bool>("semantic/display_voxel_enabled", semantic_config.display_voxel_enabled, true);
    nh.param<double>("semantic/display_voxel_size", semantic_config.display_voxel_size, 0.2);
    nh.param<int>("semantic/max_display_points", semantic_config.max_display_points, 150000);

    std::vector<int> default_color;
    std::vector<int> vio_color;
    if (nh.getParam("semantic/default_color", default_color) && default_color.size() == 3) {
        semantic_config.default_r = default_color[0];
        semantic_config.default_g = default_color[1];
        semantic_config.default_b = default_color[2];
    }
    if (nh.getParam("semantic/vio_color", vio_color) && vio_color.size() == 3) {
        semantic_config.vio_r = vio_color[0];
        semantic_config.vio_g = vio_color[1];
        semantic_config.vio_b = vio_color[2];
    }

    semantic_config.min_project_depth = std::max(1e-3, semantic_config.min_project_depth);
    semantic_config.max_project_depth = std::max(semantic_config.min_project_depth + 1e-3, semantic_config.max_project_depth);
    semantic_config.display_voxel_size = std::max(1e-3, semantic_config.display_voxel_size);
    semantic_config.max_display_points = std::max(1000, semantic_config.max_display_points);
    return true;
}

bool LoadPcdSaveConfig(ros::NodeHandle& nh, PcdSaveConfig& pcd_save_config) {
    nh.param<bool>("pcd_save/save_lidar", pcd_save_config.save_lidar, true);
    nh.param<bool>("pcd_save/save_rgb", pcd_save_config.save_rgb, false);
    nh.param<std::string>("pcd_save/output_dir", pcd_save_config.output_dir, std::string("Log/PCD"));
    nh.param<std::string>("pcd_save/raw_filename", pcd_save_config.raw_filename, std::string("all_raw_points.pcd"));
    nh.param<std::string>("pcd_save/rgb_filename", pcd_save_config.rgb_filename, std::string("all_rgb_points.pcd"));
    nh.param<bool>("pcd_save/create_dir_if_missing", pcd_save_config.create_dir_if_missing, true);
    return true;
}

bool LoadDebugVisConfig(ros::NodeHandle& nh, DebugVisConfig& debug_vis_config) {
    nh.param<bool>("debug_vis/publish_img_with_point", debug_vis_config.publish_img_with_point, true);
    nh.param<int>("debug_vis/max_points", debug_vis_config.max_points, 3000);
    nh.param<int>("debug_vis/point_radius", debug_vis_config.point_radius, 2);
    debug_vis_config.max_points = std::max(100, debug_vis_config.max_points);
    debug_vis_config.point_radius = std::max(1, debug_vis_config.point_radius);
    return true;
}

}  // namespace

LIO::LIO()
{
    FullCloudPtr scan_undistort_{new FullPointCloudType()};
    
    nh.param<std::string> ("common/lidar_topic",LiDAR_pointcloud_topic,std::string("/livox/lidar"));
    nh.param<std::string> ("common/img_topic",IMAGE_color,std::string("/camera/color/image_raw"));
    nh.param<std::string> ("common/imu_topic",IMU_topic,std::string("/livox/imu"));
    nh.param<int>("common/img_en", img_en, 1);
    nh.param<int>("common/lidar_en", lidar_en, 1);
    visual_enabled_ = (img_en != 0);

    if (visual_enabled_) {
        sub_img = nh.subscribe(IMAGE_color.c_str(), 1000000, &LIO::image_callback, this, ros::TransportHints().tcpNoDelay());
    } else {
        ROS_WARN("common/img_en == 0, visual pipeline (image callback + VIO + semantic map topics) is disabled.");
    }
    sub_pcl = nh.subscribe(LiDAR_pointcloud_topic.c_str(), 1000, &LIO::feat_points_cbk, this, ros::TransportHints().tcpNoDelay());
    sub_imu = nh.subscribe(IMU_topic.c_str(), 200000, &LIO::imu_cbk, this, ros::TransportHints().tcpNoDelay());

    nh.param<double> ("imu/init_time_seconds", init_time_seconds, 5.0);
    nh.param<int> ("imu/init_imu_queue_max_size", init_imu_queue_max_size, 600);
    nh.param<int> ("imu/static_odom_pulse", static_odom_pulse, 5);
    nh.param<double> ("imu/max_static_gyro_var", max_static_gyro_var, 0.5);
    nh.param<double> ("imu/max_static_acce_var", max_static_acce_var, 0.05);
    nh.param<double> ("imu/gravity_norm", gravity_norm, 9.81);
    nh.param<bool> ("imu/use_speed_for_static_checking", use_speed_for_static_checking, false);
    imu_init_.Setoptions(init_time_seconds, 
                         init_imu_queue_max_size, 
                         static_odom_pulse, 
                         max_static_gyro_var, 
                         max_static_acce_var, 
                         gravity_norm, 
                         use_speed_for_static_checking);
    

    nh.param<int>("ndt/max_iteration", max_iteration, 4);
    nh.param<double>("ndt/voxel_size", voxel_size, 1.0);
    nh.param<double>("ndt/inv_voxel_size", inv_voxel_size, 1.0);
    nh.param<int>("ndt/min_effective_pts", min_effective_pts, 10);
    nh.param<int>("ndt/min_pts_in_voxel", min_pts_in_voxel, 5);
    nh.param<int>("ndt/max_pts_in_voxel", max_pts_in_voxel, 50);
    nh.param<double>("ndt/eps", eps, 1e-3);
    nh.param<double>("ndt/res_outlier_th", res_outlier_th, 5.0);
    nh.param<int>("ndt/capacity", capacity, 100000);

    ndt_.Setoptions(max_iteration,
                    voxel_size,
                    inv_voxel_size,
                    min_effective_pts,
                    min_pts_in_voxel,
                    max_pts_in_voxel,
                    eps,
                    res_outlier_th,
                    capacity);

    nh.param<int>("eskf/num_iterations",  num_iterations, 3);
    nh.param<double>("eskf/quit_eps",  quit_eps, 1e-3);
    nh.param<double>("eskf/imu_dt",  imu_dt, 0.01);
    nh.param<double>("eskf/gyro_var",  gyro_var, 1e-5);
    nh.param<double>("eskf/acce_var",  acce_var, 1e-2);
    nh.param<double>("eskf/bias_gyro_var",  bias_gyro_var, 1e-6);
    nh.param<double>("eskf/bias_acce_var",  bias_acce_var, 1e-4);
    nh.param<bool>("eskf/update_bias_gyro",  update_bias_gyro, true);
    nh.param<bool>("eskf/update_bias_acce",  update_bias_acce, true);

    ieskf_.SetOptions(
        num_iterations,
        quit_eps,
        imu_dt,
        gyro_var,
        acce_var,
        bias_gyro_var,
        bias_acce_var,
        update_bias_gyro,
        update_bias_acce
    );
    cout << "\033[31;1m======= Summary of subscribed topics =======\033[0m" << endl;
    cout << "\033[31;1mLiDAR pointcloud topic: " << LiDAR_pointcloud_topic << "\033[0m" << endl;
    cout << "\033[31;1mIMU topic: " << IMU_topic << "\033[0m" << endl;
    cout << "\033[31;1mImage topic: " << IMAGE_color << "\033[0m" << endl;
    cout << "\033[31;1m=======        -End-                =======\033[0m" << endl;

    pub_img = nh.advertise<sensor_msgs::Image>("/pub_img",1000);
    pub_img_with_point = nh.advertise<sensor_msgs::Image>("/pub_img_with_point", 1000);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/mid360/orin",1000);
    pub_pcl_un = nh.advertise<sensor_msgs::PointCloud2>("/mid360/undistort",1000);
    pub_pcl_ndt = nh.advertise<sensor_msgs::PointCloud2>("/mid360/ndt_clod",1000);
    pub_map_point_rgb_ = nh.advertise<sensor_msgs::PointCloud2>("/map/map_point_rgb", 1000);
    pub_map_semantic_ = nh.advertise<sensor_msgs::PointCloud2>("/vio/map_semantic", 1000);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_map", 10);
    pubPath = nh.advertise<nav_msgs::Path>("/path", 10);
    mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    slam_mode_ = (visual_enabled_ && lidar_en) ? LIVO : ONLY_LIO;

    Eigen::Matrix3d R_i_l = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_i_l = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_c_l = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_c_l = Eigen::Vector3d::Zero();

    if (!LoadMatrix3dParam(nh, "extrinsics/T_i_l/rotation", R_i_l) ||
        !LoadVector3dParam(nh, "extrinsics/T_i_l/translation", t_i_l) ||
        !LoadMatrix3dParam(nh, "extrinsics/T_c_l/rotation", R_c_l) ||
        !LoadVector3dParam(nh, "extrinsics/T_c_l/translation", t_c_l) ||
        !LoadCameraConfig(nh, camera_config_) ||
        !LoadVioConfig(nh, vio_config_) ||
        !LoadSemanticConfig(nh, semantic_config_) ||
        !LoadPcdSaveConfig(nh, pcd_save_config_) ||
        !LoadDebugVisConfig(nh, debug_vis_config_)) {
        throw std::runtime_error("Failed to load sensor configuration from ROS parameters");
    }

    if (!visual_enabled_) {
        semantic_config_.enabled = false;
        semantic_config_.publish_map_point_rgb = false;
        semantic_config_.publish_map_semantic = false;
        debug_vis_config_.publish_img_with_point = false;
        pcd_save_config_.save_rgb = false;
    }

    T_i_l = SE3(R_i_l, t_i_l);
    T_l_c = SE3(R_c_l.transpose(), -R_c_l.transpose() * t_c_l);

    vio_manager.reset(new VIO());
    vio_manager->nav_state_w_i_ptr_ = &nav_state_w_i_;
    vio_manager->setImuToLidarExtrinsic(t_i_l, R_i_l);
    vio_manager->setLidarToCameraExtrinsic(R_c_l, t_c_l);

    vio_manager->initializeVIO();
    vio_manager->setCameraParameter(camera_config_);
    vio_manager->setVioParameter(vio_config_);
}


LIO::~LIO()
{
    
}

void LIO::feat_points_cbk(  const livox_ros_driver2::CustomMsg::ConstPtr &msg  )
{
    FullPointCloudType cloud_full ,pl_full;  // 输出点云
    mtx_buffer.lock();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer_ndt.clear();
    }

    uint plsize = msg->point_num;
    cloud_full.resize( plsize );
    for ( uint i = 1; i < plsize; i++ )
    {
        if ( ( msg->points[ i ].line < 4 ) 
            && ( !IS_VALID( msg->points[ i ].x ) ) 
            && ( !IS_VALID( msg->points[ i ].y ) ) 
            && ( !IS_VALID( msg->points[ i ].z ) )
             )   //&& msg->points[ i ].x > 0.7
        {
            if ( ( msg->points[ i ].x > 2.0 )
                && ( ( ( msg->points[ i ].tag & 0x03 ) != 0x00 )  ||  ( ( msg->points[ i ].tag & 0x0C ) != 0x00 )  || ( ( msg->points[ i ].tag & 0x30 ) != 0x00 ))
                )
            {
                continue;
            }

             float dis_i = sqrt(msg->points[i].x*msg->points[i].x +msg->points[i].y*msg->points[i].y+msg->points[i].z*msg->points[i].z);

            if((dis_i <1 && msg->points[i].x>-0.35 && msg->points[i].x < 0.35 && msg->points[i].y < -0.05 && msg->points[i].y > -0.5
            && msg->points[i].z < 1.1)  )
            {
                continue;
            }

            cloud_full[i].x = msg->points[i].x;
            cloud_full[i].y = msg->points[i].y;
            cloud_full[i].z = msg->points[i].z;
            cloud_full[i].intensity = msg->points[i].reflectivity;
            cloud_full[i].offset_time = msg->points[i].offset_time / float(1000000);
            pl_full.push_back(cloud_full[i]);
        }

    }

    FullCloudPtr ptr_ndt(new FullPointCloudType());
    *ptr_ndt = pl_full;

    if (ptr_ndt->empty()) {
            return;
    }

    lidar_buffer_ndt.push_back(ptr_ndt);

    lid_header_time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();

    mtx_buffer.unlock();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( pl_full, output );
    
    output.header.frame_id = "map";
    output.header.stamp = msg->header.stamp;
    pub_pcl.publish( output );
    
}

void LIO::imu_cbk(const sensor_msgs::ImuConstPtr &msg_in){

    if (last_timestamp_lidar < 0.0) return;

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in)); // TODO: add time offset

    double timestamp = msg->header.stamp.toSec();

    if (fabs(last_timestamp_lidar - timestamp) > 0.5 )
    {
        ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
    }
    
    msg->header.stamp = ros::Time().fromSec(timestamp);
    mtx_buffer.lock();

    if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
    {
        mtx_buffer.unlock();
        ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
        return;
    }

    if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
    {

        ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
        mtx_buffer.unlock();
        return;
    }

    last_timestamp_imu = timestamp;

    IMUPtr imu;
    imu =std::make_shared<IMU>(msg->header.stamp.toSec(),
                                Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                Eigen::Vector3d(msg->linear_acceleration.x * 9.80665, msg->linear_acceleration.y * 9.80665,
                                    msg->linear_acceleration.z * 9.80665  ));
    imu_buffer.push_back(imu);
    mtx_buffer.unlock();

    return;
}

void LIO::image_callback( const sensor_msgs::ImageConstPtr &msg ){
    if (!visual_enabled_) {
        return;
    }


    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
    double msg_header_time = msg->header.stamp.toSec(); //+ img_time_offset;
    if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
    if (last_timestamp_lidar < 0) return;

    if (msg_header_time < last_timestamp_img)
    {
        ROS_ERROR("image loop back. \n");
        return;
    }
    mtx_buffer.lock();
    double img_time_correct = msg_header_time;

    if (img_time_correct - last_timestamp_img < 0.02)
    {
        ROS_WARN("Image need Jumps: %.6f", img_time_correct);
        mtx_buffer.unlock();
        return;
    }

    cv::Mat image_cur = cv_ptr_compressed->image;
    last_rgb_img_ = image_cur.clone();
    last_rgb_stamp_ = img_time_correct;
    cv_ptr_compressed->image.release();
    img_buffer.push_back(image_cur);
    img_time_buffer.push_back(img_time_correct);

    last_timestamp_img = img_time_correct;
    mtx_buffer.unlock();

}

bool LIO::sync_packages(LidarMeasureGroup &meas)
{
    if(lidar_buffer_ndt.empty() && lidar_en) return false;
    if(img_buffer.empty() && visual_enabled_) return false;

    switch (slam_mode_)
    {
    case ONLY_LIO:
    {
        ROS_INFO("-----------ONLY_LIO MODE!!!");
        if (lidar_buffer_ndt.empty()) return false;
        ROS_INFO("-----------ONLY_LIO MODE!!! 1");
        if (imu_buffer.empty()) return false;
ROS_INFO("-----------ONLY_LIO MODE!!! 1.1");
        if (!lidar_pushed)
        {
            // If not push the lidar into measurement data buffer
            // 激光雷达点云指针缓存器的第一帧点云给 给雷达测量组meas.lidar
            meas.lidar = lidar_buffer_ndt.front(); // push the first lidar topic
            ROS_INFO("-----------ONLY_LIO MODE!!! 1.3");
            if (meas.lidar->points.size() <= 1) return false;  // 判断点云数量
            ROS_INFO("-----------ONLY_LIO MODE!!! 1.2");
            meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
            meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().offset_time / double(1000); // calc lidar scan end time
            meas.pcl_proc_cur = meas.lidar;
            
            meas.pcl_proc_cur = lidar_buffer_ndt.front();
            lidar_pushed = true;                                                                                       // flag
        }
        ROS_INFO("-----------ONLY_LIO MODE!!! 1.3");
        // TODO: add imu data to meas
        if (last_timestamp_imu < meas.lidar_frame_end_time)
        { 
        return false;
        }
ROS_INFO("-----------ONLY_LIO MODE!!! 2");
        struct MeasureGroup m;
        //m.imu.clear();
        m.imu.clear();
        m.lio_time = meas.lidar_frame_end_time;

        mtx_buffer.lock();
        while (!imu_buffer.empty())
        {
        if (imu_buffer.front()->timestamp_ > meas.lidar_frame_end_time) break;
        //IMUPtr imu = imuIMU(imu_buffer.front());
        IMUPtr imu = imu_buffer.front();
        m.imu.emplace_back(imu);
        //   ROS_INFO("imu->timestamp_: %.6f ",  imu->timestamp_);
        //m.imu.push_back(imu_buffer.front()); // 添加IMU数据
        imu_buffer.pop_front(); // 移除处理过的IMU数据
        }
        ROS_INFO("-----------ONLY_LIO MODE!!! 3");
        // 移除激光雷达数据和时间戳
        // lid_raw_data_buffer.pop_front();
        lidar_buffer_ndt.pop_front();
        lid_header_time_buffer.pop_front();
        mtx_buffer.unlock();

        // meas.lio_vio_flg = LIO; // 标记为LIO模式
        meas.measures.push_back(m); // 将测量数据添加到队列中
        lidar_pushed = false; // 重置标记
        ROS_INFO("-----------ONLY_LIO MODE!!! end");
        return true;
        
        break;
    }
    case LIVO:
    {
        // ROS_INFO("-----------LIVO MODE!!!");
        double img_capture_time = img_time_buffer.front();
        if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
        // ROS_INFO("[sync]--------->last_lio_update_time= %.06lf",meas.last_lio_update_time);
        double lid_newest_time = lid_header_time_buffer.back() + lidar_buffer_ndt.back()->points.back().curvature / double(1000);
        double imu_newest_time = imu_buffer.back()->timestamp_;

        if (img_capture_time < meas.last_lio_update_time + 0.00001)
        {
            img_buffer.pop_front();
            img_time_buffer.pop_front();
            ROS_ERROR("[ Data Cut ] Throw one image frame! \n");
            return false;
        }

        if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
        {
            return false;
        }

        struct MeasureGroup m;

        // 加入imu数据
        m.imu.clear();
        m.lio_time = img_capture_time;

        mtx_buffer.lock();
        while (!imu_buffer.empty())
        {
            if (imu_buffer.front()->timestamp_ > m.lio_time) break;

            if (imu_buffer.front()->timestamp_ > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

            imu_buffer.pop_front();

        }
        mtx_buffer.unlock();
        ROS_INFO("-----------LIVO MODE  add imu");

        *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
        // ROS_INFO("-----------LIVO MODE  add lidar1");
        FullPointCloudType().swap(*meas.pcl_proc_next);
        // ROS_INFO("-----------LIVO MODE  add lidar2");
        // 加入lidar数据
        int lid_frame_num = lidar_buffer_ndt.size();
        int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
        meas.pcl_proc_cur->reserve(max_size);
        meas.pcl_proc_next->reserve(max_size);
        // ROS_INFO("-----------LIVO MODE  add lidar3");
        // meas.last_lio_update_time = lid_header_time_buffer.front();
        
        while (!lidar_buffer_ndt.empty())
        {
            if (lid_header_time_buffer.empty()) {
                // 处理空队列情况
                break; // 或其他适当的错误处理
            }
            mtx_buffer.lock();
            // ROS_INFO("-----------while (!lidar_buffer_ndt.empty())---- 0");
            if (lid_header_time_buffer.front() > m.lio_time){
                mtx_buffer.unlock();break;
            } 
            // ROS_INFO("-----------while (!lidar_buffer_ndt.empty())---- 1");
            auto pcl(lidar_buffer_ndt.front()->points);
            // ROS_INFO("-----------while (!lidar_buffer_ndt.empty())---- 2");
            double frame_header_time(lid_header_time_buffer.front());
            // ROS_INFO("-----------while (!lidar_buffer_ndt.empty())---- 3");
            double max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;
            // ROS_INFO("[sync]--------->max_offs_time_ms= %.06lf",max_offs_time_ms);
            mtx_buffer.unlock();

            for (int i = 0; i < pcl.size(); i++)
            {
            auto pt = pcl[i];
            if (pcl[i].offset_time < max_offs_time_ms)
            {
                pt.offset_time += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
                // pt.offset_time = pt.curvature / double(1000);
                meas.pcl_proc_cur->points.push_back(pt);
            }
            else
            {
                pt.offset_time += (frame_header_time - m.lio_time) * 1000.0f;
                // pt.offset_time = pt.curvature / double(1000);
                meas.pcl_proc_next->points.push_back(pt);
            }
            }
            mtx_buffer.lock();
            lidar_buffer_ndt.pop_front();
            lid_header_time_buffer.pop_front();
            mtx_buffer.unlock();
        }
        
        // ROS_INFO("[sync]--------->pt.offset_time= %.06lf",meas.pcl_proc_cur->points.back().offset_time);
        // meas.last_lio_update_time = meas.pcl_proc_cur
        // meas.pcl_proc_cur_ndt = meas.pcl_proc_cur;
        // meas.last_lio_update_time += meas.pcl_proc_cur->points.back().offset_time;
        //  ROS_INFO("-----------while (!lidar_buffer_ndt.empty())---- 4");
        meas.lidar_frame_beg_time = meas.last_lio_update_time;                                                // generate lidar_frame_beg_time
        meas.lidar_frame_end_time = m.lio_time;
        //  ROS_INFO("-----------while (!lidar_buffer_ndt.empty())---- 5");
        meas.last_lio_update_time = m.lio_time; // m.lio_time;
        // ROS_INFO("-----------while (!lidar_buffer_ndt.empty())---- 6");
        // meas.measures.push_back(m);

        // 加入camera数据
        // m.lio_time = meas.last_lio_update_time;
        m.vio_time = img_capture_time;
        // ROS_INFO("-----------while (!lidar_buffer_ndt.empty())---- 7");
        m.img = img_buffer.front();
        ROS_INFO("-----------LIVO MODE  add camera");

        mtx_buffer.lock();
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        mtx_buffer.unlock();
        meas.measures.push_back(m);

        return true;
    }

    default:
    {
        return false;
    }
    break;
        
    }

    
}

void LIO::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    // p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}


void LIO::ProcessIMU()
{
    ROS_INFO("ProcessIMU....");
    // imu init
    if (imu_need_init_) {
        // 初始化IMU系统
        ROS_INFO("start init---->");
        TryInitIMU();
        return;
    }

    // predict imu state
    imu_states_.clear();
    imu_states_.emplace_back(ieskf_.GetNominalState());

    for (auto &imu : LidarMeasures.measures.back().imu)
    {
        ieskf_.Predict(*imu);
        imu_states_.emplace_back(ieskf_.GetNominalState());
    }
    ROS_INFO("imu_states_: x=%.3f,y=%.3f,z=%.3f", imu_states_.back().p_[0],imu_states_.back().p_[1],imu_states_.back().p_[2]);
}

void LIO::TryInitIMU()
{
    for (auto imu : LidarMeasures.measures.back().imu) {
        imu_init_.AddIMU(*imu);
    }
    
    if(imu_init_.InitSuccess())
    {
        IESKFD::Options options;
        options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
        options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
        ieskf_.SetInitialConditions(options,imu_init_.GetInitBg(),imu_init_.GetInitBa(),imu_init_.GetGravity());
        imu_need_init_ = false;
        ROS_INFO("IMU init success ---->");
        // ROS_INFO("-----------------------------------------------------------");
    }
    
}

void LIO::Undistort()
{
    // scan_undistort_.clear();
    auto cloud = LidarMeasures.pcl_proc_cur;
    auto imu_state = ieskf_.GetNominalState();
    SE3 T_end = SE3(imu_state.R_, imu_state.p_);
    // ROS_INFO("[ProcessLidar]--------->Undistort 1");
    //std::execution::par_unseq, 并发模式 c++17以上
    double pt_time=0.0;
    std::for_each( cloud->points.begin(), cloud->points.end(),[&](auto &pt){
        SE3 Ti = T_end;
        NavStated match;
        // auto time_p = pt.curvature / double(1000);
        //LidarMeasures.lidar_frame_beg_time + pt.offset_time * 1e-3
        //* 1e-3
        pt_time = LidarMeasures.lidar_frame_beg_time + pt.offset_time * 1e-3;
        
        PoseInterp<NavStated>(
            pt_time , imu_states_, [](const NavStated &s) { return s.timestamp_; },
            [](const NavStated &s) { return s.GetSE3(); }, Ti, match);

        Eigen::Vector3d pi = ToVec3d(pt);
        Eigen::Vector3d p_compensate = T_i_l.inverse() * T_end.inverse() * Ti * T_i_l * pi;
        // ROS_INFO("[ProcessLidar]--------->Undistort 3");
        pt.x = p_compensate(0);
        pt.y = p_compensate(1);
        pt.z = p_compensate(2);

    });
    // LidarMeasures.last_lio_update_time = pt_time;
    // ROS_INFO("[ProcessLidar]--------->pt_time= %.lf",pt_time);
    // ROS_INFO("[ProcessLidar]--------->Undistort 4");
    scan_undistort_ = cloud;
    // TODO: 需要解决scan_undistort_、cloud数据类型不匹配的问题；

    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg( *cloud, output );
    // output.header.frame_id = "map";
    // // output.header.stamp = ros::Time::fromSec(LidarMeasures.lidar_frame_end_time);
    
    // // std::cout<<"360 frame id : "<< output.header.frame_id << endl;
    // pub_pcl_un.publish( output );

}

void LIO::Align()
{
    if (scan_undistort_->empty()) return;
    FullCloudPtr scan_undistort_trans(new FullPointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, T_i_l.matrix().cast<float>());
    scan_undistort_ = scan_undistort_trans;
    
    current_scan_ = ConvertToCloud<FullPointType>(scan_undistort_);

    // voxel 之
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(0.5, 0.5, 0.5);
    voxel.setInputCloud(current_scan_);

    CloudPtr current_scan_filter(new PointCloudType);
    voxel.filter(*current_scan_filter);

    if (flg_first_scan_lio) {
        ndt_.AddCloud(current_scan_);
        flg_first_scan_lio = false;

        return;
    }

    // 后续的scan，使用NDT配合pose进行更新
    // LOG(INFO) << "=== frame " << frame_num_;

    ndt_.SetSource(current_scan_filter);
    ieskf_.UpdateUsingCustomObserve([this](const SE3 &input_pose, Mat18d &HTVH, Vec18d &HTVr) {
        ndt_.ComputeResidualAndJacobians(input_pose, HTVH, HTVr);
    });

    auto current_nav_state = ieskf_.GetNominalState();
    nav_state_w_i_ = current_nav_state;
    // 若运动了一定范围，则把点云放入地图中
    SE3 T_w_i = ieskf_.GetNominalSE3();
    SE3 delta_pose = last_keyframe_T_w_i_.inverse() * T_w_i;
    
    // CloudPtr current_scan_world(new PointCloudType);
    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr current_scan_world{new pcl::PointXYZINormal};
    pcl::transformPointCloud(*current_scan_filter, *current_scan_world, T_w_i.matrix());
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_, T_w_i.matrix());

    sensor_msgs::PointCloud2 output1;
    pcl::toROSMsg( *scan_undistort_, output1 );
    output1.header.frame_id = "map";
    // output.header.stamp = ros::Time::fromSec(LidarMeasures.lidar_frame_end_time);
    
    // std::cout<<"360 frame id : "<< output.header.frame_id << endl;
    pub_pcl_un.publish( output1 );

    *pcl_wait_save += *current_scan_world;
    vio_manager->vio_scan_world = current_scan_world;
    if (visual_enabled_) {
        BuildMapPointRGB(current_scan_world, T_w_i);
        BuildSemanticHighlightCloud(vio_manager ? vio_manager->new_frame_ : FramePtr());
        PublishSemanticClouds();
        if (pcd_save_config_.save_rgb && map_point_rgb_ && !map_point_rgb_->empty()) {
            *rgb_wait_save_ += *map_point_rgb_;
            if (rgb_wait_save_->size() > 3000000) {
                UiCloudPtr compact(new UiPointCloudType);
                DownsampleUiCloud(rgb_wait_save_, compact, semantic_config_.display_voxel_size, 2000000);
                rgb_wait_save_.swap(compact);
            }
        }
    } else {
        map_point_rgb_->clear();
        map_semantic_->clear();
    }
    
    // pcl::concatenate(*pcl_wait_save, *current_scan_world, *pcl_wait_save);

    if (delta_pose.translation().norm() > 1.0 || delta_pose.so3().log().norm() > deg2rad(10.0)) {
        // 将地图合入NDT中
        
        ndt_.AddCloud(current_scan_world);
        last_keyframe_T_w_i_ = T_w_i;
    }
    
    publish_odometry(pubOdomAftMapped);
    publish_path(pubPath);
    publish_mavros(mavros_pose_publisher);
    // pcl::transformPointCloud(*current_scan_filter, *current_scan_w, current_pose.matrix());

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( *current_scan_world, output );
    output.header.frame_id = "map";
    // output.header.stamp = ros::Time::fromSec(LidarMeasures.lidar_frame_end_time);
    
    // std::cout<<"360 frame id : "<< output.header.frame_id << endl;
    pub_pcl_ndt.publish( output );

    frame_num_++;
    return;

}

void LIO::ProcessLidar()
{
    if (imu_need_init_) {
        return;
    }
    // ROS_INFO("[ProcessLidar]--------->imu_need_init_");
    // lildar undistort
    Undistort();
    // ROS_INFO("[ProcessLidar]--------->Undistort");
    Align();
    // ROS_INFO("[ProcessLidar]--------->Align");
    return;
}

bool LIO::ProjectWorldPointToImage(const Eigen::Vector3d& pt_w, const SE3& T_c_w, Eigen::Vector2d& uv, double& depth) const
{
    if (!vio_manager) {
        return false;
    }
    Eigen::Vector3d pt_c = T_c_w * pt_w;
    depth = pt_c.z();
    if (depth <= 0.0 || depth < semantic_config_.min_project_depth || depth > semantic_config_.max_project_depth) {
        return false;
    }
    uv = vio_manager->cameraPointToPixel(pt_c);
    return vio_manager->isUVEffective(uv);
}

UiPointType LIO::MakeRGBA(const Eigen::Vector3d& p, int r, int g, int b, int a) const
{
    UiPointType out;
    out.x = p.x();
    out.y = p.y();
    out.z = p.z();
    out.r = static_cast<uint8_t>(std::max(0, std::min(255, r)));
    out.g = static_cast<uint8_t>(std::max(0, std::min(255, g)));
    out.b = static_cast<uint8_t>(std::max(0, std::min(255, b)));
    out.a = static_cast<uint8_t>(std::max(0, std::min(255, a)));
    return out;
}

void LIO::BuildMapPointRGB(const CloudPtr& scan_world, const SE3& T_w_i)
{
    if (!semantic_config_.enabled || !scan_world || !vio_manager) {
        return;
    }

    // Keep the semantic visualization in per-frame mode to match /mid360/ndt_clod behavior.
    map_point_rgb_->clear();
    map_point_rgb_->points.reserve(scan_world->points.size());

    const bool has_rgb = !last_rgb_img_.empty() && last_rgb_img_.channels() == 3;
    if (!has_rgb) {
        map_point_rgb_->is_dense = false;
        map_point_rgb_->width = 0;
        map_point_rgb_->height = 1;
        return;
    }
    const SE3 T_c_w = vio_manager->T_c_i * T_w_i.inverse();

    for (const auto& pt : scan_world->points) {
        const Eigen::Vector3d p_w(pt.x, pt.y, pt.z);
        Eigen::Vector2d uv;
        double depth = 0.0;
        if (!ProjectWorldPointToImage(p_w, T_c_w, uv, depth)) {
            continue;
        }
        const int u = static_cast<int>(std::round(uv.x()));
        const int v = static_cast<int>(std::round(uv.y()));
        if (u < 0 || u >= last_rgb_img_.cols || v < 0 || v >= last_rgb_img_.rows) {
            continue;
        }
        const cv::Vec3b color = last_rgb_img_.at<cv::Vec3b>(v, u);
        const int b = static_cast<int>(color[0]);
        const int g = static_cast<int>(color[1]);
        const int r = static_cast<int>(color[2]);
        map_point_rgb_->points.emplace_back(MakeRGBA(p_w, r, g, b, 255));
    }

    map_point_rgb_->is_dense = false;
    map_point_rgb_->width = static_cast<uint32_t>(map_point_rgb_->points.size());
    map_point_rgb_->height = 1;
    if (semantic_config_.display_voxel_enabled && !map_point_rgb_->empty()) {
        UiCloudPtr compact(new UiPointCloudType);
        DownsampleUiCloud(map_point_rgb_, compact, semantic_config_.display_voxel_size, semantic_config_.max_display_points);
        map_point_rgb_.swap(compact);
    }
}

void LIO::BuildSemanticHighlightCloud(const FramePtr& frame_ref)
{
    if (!semantic_config_.enabled) {
        return;
    }
    map_semantic_->clear();
    *map_semantic_ = *map_point_rgb_;
    if (!semantic_config_.highlight_vio_points || !frame_ref) {
        return;
    }

    for (const auto& pt : frame_ref->pts) {
        map_semantic_->points.emplace_back(
            MakeRGBA(pt.pt_w, semantic_config_.vio_r, semantic_config_.vio_g, semantic_config_.vio_b, 255));
    }
    map_semantic_->is_dense = false;
    map_semantic_->width = static_cast<uint32_t>(map_semantic_->points.size());
    map_semantic_->height = 1;
    if (semantic_config_.display_voxel_enabled && !map_semantic_->empty()) {
        UiCloudPtr compact(new UiPointCloudType);
        DownsampleUiCloud(map_semantic_, compact, semantic_config_.display_voxel_size, semantic_config_.max_display_points);
        map_semantic_.swap(compact);
    }
}

void LIO::DownsampleUiCloud(const UiCloudPtr& in, UiCloudPtr& out, double voxel_size, int max_points) const
{
    if (!in) {
        return;
    }
    pcl::VoxelGrid<UiPointType> voxel;
    voxel.setLeafSize(static_cast<float>(voxel_size), static_cast<float>(voxel_size), static_cast<float>(voxel_size));
    voxel.setInputCloud(in);
    voxel.filter(*out);

    if (out->size() > static_cast<size_t>(max_points)) {
        UiCloudPtr sampled(new UiPointCloudType);
        sampled->points.reserve(max_points);
        const double step = static_cast<double>(out->size()) / static_cast<double>(max_points);
        for (int i = 0; i < max_points; ++i) {
            sampled->points.push_back(out->points[static_cast<size_t>(i * step)]);
        }
        sampled->is_dense = false;
        sampled->width = static_cast<uint32_t>(sampled->points.size());
        sampled->height = 1;
        out.swap(sampled);
    } else {
        out->is_dense = false;
        out->width = static_cast<uint32_t>(out->points.size());
        out->height = 1;
    }
}

void LIO::PublishSemanticClouds()
{
    if (!semantic_config_.enabled) {
        return;
    }
    if (semantic_config_.publish_map_point_rgb && pub_map_point_rgb_ && !map_point_rgb_->empty()) {
        sensor_msgs::PointCloud2 msg_rgb;
        pcl::toROSMsg(*map_point_rgb_, msg_rgb);
        msg_rgb.header.frame_id = "map";
        msg_rgb.header.stamp = ros::Time::now();
        pub_map_point_rgb_.publish(msg_rgb);
    }

    if (semantic_config_.publish_map_semantic && pub_map_semantic_ && !map_semantic_->empty()) {
        sensor_msgs::PointCloud2 msg_semantic;
        pcl::toROSMsg(*map_semantic_, msg_semantic);
        msg_semantic.header.frame_id = "map";
        msg_semantic.header.stamp = ros::Time::now();
        pub_map_semantic_.publish(msg_semantic);
    }
}

void LIO::publishImageWithProjectedPoints()
{
    if (!visual_enabled_ || !debug_vis_config_.publish_img_with_point || !vio_manager || !vio_manager->new_frame_) {
        return;
    }
    if (vio_manager->new_frame_->img_.empty()) {
        return;
    }

    cv::Mat img_vis = vio_manager->new_frame_->img_.clone();
    int draw_count = 0;
    const auto& selected_pts = vio_manager->new_frame_->pts;
    const size_t total_points = selected_pts.size();
    if (total_points == 0) {
        return;
    }
    const size_t step = std::max<size_t>(1, total_points / static_cast<size_t>(debug_vis_config_.max_points));

    for (size_t i = 0; i < total_points; i += step) {
        const auto& pt = selected_pts[i];
        const int u = static_cast<int>(std::round(pt.px_ref.x()));
        const int v = static_cast<int>(std::round(pt.px_ref.y()));
        if (u < 0 || u >= img_vis.cols || v < 0 || v >= img_vis.rows) {
            continue;
        }
        cv::circle(img_vis, cv::Point(u, v), debug_vis_config_.point_radius, cv::Scalar(255), -1);
        draw_count++;
        if (draw_count >= debug_vis_config_.max_points) {
            break;
        }
    }

    cv::putText(img_vis, "VIO init pts: " + std::to_string(draw_count), cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255), 2);

    cv_bridge::CvImage msg_out;
    msg_out.header.stamp = ros::Time::now();
    msg_out.header.frame_id = "map";
    msg_out.encoding = sensor_msgs::image_encodings::MONO8;
    msg_out.image = img_vis;
    pub_img_with_point.publish(msg_out.toImageMsg());
}

void LIO::savePCD()
{
    if ((!pcd_save_config_.save_lidar || !pcl_wait_save || pcl_wait_save->empty()) &&
        (!visual_enabled_ || !pcd_save_config_.save_rgb || !rgb_wait_save_ || rgb_wait_save_->empty())) {
        return;
    }

    const std::filesystem::path output_dir = pcd_save_config_.output_dir.empty()
        ? std::filesystem::path("Log/PCD")
        : std::filesystem::path(pcd_save_config_.output_dir);
    const std::filesystem::path raw_points_path = output_dir / pcd_save_config_.raw_filename;
    const std::filesystem::path rgb_points_path = output_dir / pcd_save_config_.rgb_filename;

    if (pcd_save_config_.create_dir_if_missing) {
        std::error_code ec;
        std::filesystem::create_directories(output_dir, ec);
        if (ec) {
            ROS_ERROR_STREAM("Failed to create PCD output directory: " << output_dir << ", error: " << ec.message());
            return;
        }
    }

    pcl::PCDWriter pcd_writer;
    
    if (pcd_save_config_.save_lidar && pcl_wait_save && pcl_wait_save->size() > 0)
    {
        pcd_writer.writeBinary(raw_points_path.string(), *pcl_wait_save);
        std::cout << "Raw point cloud data saved to: " << raw_points_path
                  << ", with point count: " << pcl_wait_save->points.size() << std::endl;
    }
    if (visual_enabled_ && pcd_save_config_.save_rgb && rgb_wait_save_ && rgb_wait_save_->size() > 0) {
        pcd_writer.writeBinary(rgb_points_path.string(), *rgb_wait_save_);
        std::cout << "RGB point cloud data saved to: " << rgb_points_path
                  << ", with point count: " << rgb_wait_save_->points.size() << std::endl;
    }
    
}

void LIO::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{

    odomAftMapped.header.frame_id = "map";
    odomAftMapped.child_frame_id = "aft_mapped";
    odomAftMapped.header.stamp = ros::Time::now(); //.ros::Time()fromSec(last_timestamp_lidar);
    set_posestamp(odomAftMapped.pose.pose);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, 
                                    odomAftMapped.pose.pose.position.y, 
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform( tf::StampedTransform(transform, odomAftMapped.header.stamp, "map", "aft_mapped") );
    pubOdomAftMapped.publish(odomAftMapped);

}

template <typename T> void LIO::set_posestamp(T &out)
{
    SE3 T_w_i = ieskf_.GetNominalSE3();
    Eigen::Vector3d position = T_w_i.matrix().block<3,1>(0,3);
    out.position.x = position.x();
    out.position.y = position.y();
    out.position.z = position.z();

    Eigen::Matrix3d rotation_matrix = T_w_i.matrix().block<3,3>(0,0);
    Eigen::Quaterniond quaternion(rotation_matrix);
    quaternion.normalize();  // 规范化四元数（确保是单位四元数）

    out.orientation.x = quaternion.x();
    out.orientation.y = quaternion.y();
    out.orientation.z = quaternion.z();
    out.orientation.w = quaternion.w();
}

void LIO::publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "/map";
  path.poses.push_back(msg_body_pose);
  pubPath.publish(path);
}

void LIO::publish_mavros(const ros::Publisher &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "/map";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher.publish(msg_body_pose);
}

void LIO::ProcessCamera()
{
    if (!visual_enabled_) return;
    if(lidar_buffer_ndt.empty() && lidar_en) return;
    if(img_buffer.empty() && visual_enabled_) return;
    // ROS_INFO("--------------------------------------------5");
    if (pcl_wait_save->empty() || (pcl_wait_save == nullptr)) 
    {
        std::cout << "[ VIO ] No point!!!" << std::endl;
        return;
    }
    ROS_INFO("--------------------------------------------6");
    initcamera();
    
    handleVIO();

}

void LIO::initcamera()
{
    
    return;
}

void LIO::handleVIO()
{
    if (!visual_enabled_ || !vio_manager) {
        return;
    }
    // if (image_get.empty()) return;
    SE3 T_w_i_meas;
    vio_manager->processFrame(LidarMeasures.measures.back().img, T_w_i_meas);
    if (!vio_manager->visual_measurement_ready_) {
        return;
    }

    ieskf_.UpdateUsingCustomObserve([this](const SE3 &input_pose, Mat18d &HTVH, Vec18d &HTVr) {
        vio_manager->ComputeResidualAndJacobians(input_pose, HTVH, HTVr);
    });
    const auto& tracking_stats = vio_manager->trackingStats();
    ROS_INFO("[VIO] frame stats: candidates=%d selected=%d valid=%d inliers=%d mean_abs=%.3f accepted=%d promote=%d",
             tracking_stats.candidate_points, tracking_stats.selected_points, tracking_stats.valid_residuals,
             tracking_stats.inlier_residuals, tracking_stats.mean_abs_residual,
             tracking_stats.update_accepted, tracking_stats.promote_reference);
    if (tracking_stats.promote_reference) {
        vio_manager->finalizeFrame();
    } else if (!tracking_stats.update_accepted) {
        vio_manager->visual_measurement_ready_ = false;
    }

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();               // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever BGR8
    out_msg.image = vio_manager->new_frame_->img_;                                   // Your cv::Mat
    out_msg.header.frame_id = "map";

    // std::cout<<"color frame id : "<< out_msg.header.frame_id << endl;
    pub_img.publish( out_msg );
    publishImageWithProjectedPoints();
    
}

void LIO::run()
{
    // This function can be used to start the processing loop if needed
    // For now, it is empty as the callbacks will handle the data processing
    ros::Rate rate(30); // 30 Hz
    while (ros::ok())
    {
        // Process any callbacks
        ros::spinOnce();
        auto time1 = std::chrono::high_resolution_clock::now(); 
        if (!sync_packages(LidarMeasures))  
        {   
            // ROS_INFO("--------------------------------------------1");
            // // std::cout <<"imu: " << LidarMeasures.measures.back().imu.back()->header.stamp.toSec() << std::endl;
            // ROS_INFO("[LidarMeasures] LidarMeasures measures.size: %ld",LidarMeasures.measures.size());
            // ROS_INFO("[LidarMeasures] LidarMeasures lidar_frame_end_time: %.6f",LidarMeasures.lidar_frame_end_time);
            // std::cout << LidarMeasures.measures.front().imu.back()->timestamp_ << std::endl;
            // ROS_INFO("[LidarMeasures] LidarMeasures imu lio time: %.6f",LidarMeasures.measures.front().lio_time);
            // LidarMeasures.measures.pop_front();
            rate.sleep();
            continue;
        }
        // ROS_INFO("--------------------------------------------2.2");
        handleFirstFrame();
        // ROS_INFO("--------------------------------------------2");
        auto time2 = std::chrono::high_resolution_clock::now(); 

        ProcessIMU();
        // ROS_INFO("--------------------------------------------3");
        
        ProcessLidar();
        // ROS_INFO("--------------------------------------------4");
        ProcessCamera();
        auto time3 = std::chrono::high_resolution_clock::now(); 
        std::chrono::duration<double> duration_time = time2 - time1;

        double duration_time_seconds = duration_time.count(); 
        std::chrono::duration<double> duration_time2 = time3 - time2;

        double duration_time_seconds2 = duration_time2.count(); 
        
        ROS_INFO("[sync_packages mode] time: %.06lf ms", duration_time_seconds * 1000);
        ROS_INFO("[ProcessIMU and ProcessLidar mode] time: %.06lf ms", duration_time_seconds2 * 1000);
    }
    savePCD();
}
