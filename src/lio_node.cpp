/*
 * @Auther: yangj
 * @Description:  
 * @Date: 2025-06-19 09:17:49
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-10-21 07:27:09
 */

#include"utils/lio_node.h"

LIO::LIO(/* args */)
{
    FullCloudPtr scan_undistort_{new FullPointCloudType()};
    
    nh.param<std::string> ("common/lidar_topic",LiDAR_pointcloud_topic,std::string("/livox/lidar"));
    nh.param<std::string> ("common/img_topic",IMAGE_color,std::string("/camera/color/image_raw"));
    nh.param<std::string> ("common/imu_topic",IMU_topic,std::string("/livox/imu"));
    nh.param<int>("common/img_en", img_en, 1);
    nh.param<int>("common/lidar_en", lidar_en, 1);

    sub_img = nh.subscribe(IMAGE_color.c_str(), 1000000, &LIO::image_callback, this, ros::TransportHints().tcpNoDelay());
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

    // ESKF 参数配置
    nh.param<int>("eskf/num_iterations",  num_iterations, 3);
    nh.param<double>("eskf/quit_eps",  quit_eps, 1e-3);
    nh.param<double>("eskf/imu_dt",  imu_dt, 0.01);
    nh.param<double>("eskf/gyro_var",  gyro_var, 1e-5);
    nh.param<double>("eskf/acce_var",  acce_var, 1e-2);
    nh.param<double>("eskf/bias_gyro_var",  bias_gyro_var, 1e-6);
    nh.param<double>("eskf/bias_acce_var",  bias_acce_var, 1e-4);
    nh.param<bool>("eskf/update_bias_gyro",  update_bias_gyro, true);
    nh.param<bool>("eskf/update_bias_acce",  update_bias_acce, true);

    // 设置ESKF参数
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



    if(1)
        {
            
            cout << "\033[31;1m======= Summary of subscribed topics =======\033[0m" << endl; 
            cout << "\033[31;1mLiDAR pointcloud topic: " << LiDAR_pointcloud_topic << "\033[0m" << endl;
            cout << "\033[31;1mIMU topic: " << IMU_topic << "\033[0m" << endl;
            cout << "\033[31;1mImage topic: " << IMAGE_color << "\033[0m" << endl;
            cout << "\033[31;1m=======        -End-                =======\033[0m" << endl;
            
        }

    pub_img = nh.advertise<sensor_msgs::Image>("/pub_img",1000);
    pub_depth_img = nh.advertise<sensor_msgs::Image>("/pub_depth_img",1000);
    pub_img_comp = nh.advertise<sensor_msgs::Image>("/pub_img/image_raw",1000);
    pub_img_comp_info = nh.advertise<sensor_msgs::CameraInfo>("/pub_img/camera_info",1000);
    pub_depth_img_comp = nh.advertise<sensor_msgs::Image>("/pub_depth_img_comp",1000);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/mid360/orin",1000);
    pub_pcl_un = nh.advertise<sensor_msgs::PointCloud2>("/mid360/undistort",1000);
    pub_pcl_ndt = nh.advertise<sensor_msgs::PointCloud2>("/mid360/ndt_clod",1000);
    pub_camera_odom = nh.advertise<nav_msgs::Odometry>("/pub_camera_odom",1000);
    pub_path = nh.advertise<nav_msgs::Path>("/pub_apriltag_path",1000);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_map", 10);
    pubPath = nh.advertise<nav_msgs::Path>("/path", 10);
    mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    // std::vector<double> ext_t = {-0.011, -0.02329, 0.04412};
    // std::vector<double> ext_r ={1, 0, 0, 0, 1, 0, 0, 0, 1};
    ext_t.assign(3, 0.0);
    ext_r.assign(9, 0.0);
    cameraextrinT.assign(3, 0.0);
    cameraextrinR.assign(9, 0.0);

    nh.param<vector<double>>("extrin_calib/extrinsic_T", ext_t, vector<double>());
    nh.param<vector<double>>("extrin_calib/extrinsic_R", ext_r, vector<double>());
    nh.param<vector<double>>("extrin_calib/Pcl", cameraextrinT, vector<double>());
    nh.param<vector<double>>("extrin_calib/Rcl", cameraextrinR, vector<double>());

    Eigen::Vector3d lidar_T_wrt_IMU = VecFromArray(ext_t);
    Eigen::Matrix3d lidar_R_wrt_IMU = MatFromArray(ext_r);
    TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
        
    Eigen::Vector3d camera_T_wrt_lidar = VecFromArray(cameraextrinT);
    Eigen::Matrix3d camera_R_wrt_lidar = MatFromArray(cameraextrinR);
    TLC_ = SE3(camera_R_wrt_lidar, camera_T_wrt_lidar);
    
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    slam_mode_ = (img_en && lidar_en) ? LIVO : ONLY_LIO;

    
}


LIO::~LIO()
{

    // output_cloud.is_dense = false;
    // output_cloud.width = output_cloud.points.size();
    // output_cloud.height = 1;

    // pcl::io::savePCDFileASCII("bagtopcd.pcd", output_cloud);
    // ROS_INFO("save pcd to ./bagtopcd.pcd");

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
        // clang-format off
        // pcl::PointXYZI p;
        if ( ( msg->points[ i ].line < 4 ) 
            && ( !IS_VALID( msg->points[ i ].x ) ) 
            && ( !IS_VALID( msg->points[ i ].y ) ) 
            && ( !IS_VALID( msg->points[ i ].z ) )
             )   //&& msg->points[ i ].x > 0.7
        {
            // https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol
            // See [3.4 Tag Information] 
            if ( ( msg->points[ i ].x > 2.0 )
                && ( ( ( msg->points[ i ].tag & 0x03 ) != 0x00 )  ||  ( ( msg->points[ i ].tag & 0x0C ) != 0x00 )  || ( ( msg->points[ i ].tag & 0x30 ) != 0x00 ))
                )
            {
                // Remove the bad quality points
                continue;
            }

             float dis_i = sqrt(msg->points[i].x*msg->points[i].x +msg->points[i].y*msg->points[i].y+msg->points[i].z*msg->points[i].z);

            if((dis_i <1 && msg->points[i].x>-0.35 && msg->points[i].x < 0.35 && msg->points[i].y < -0.05 && msg->points[i].y > -0.5
            && msg->points[i].z < 1.1)  )
            {
                // ROS_INFO("dis_i=%f;\n",dis_i);
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

    // cloud_full.height = 1;
    // cloud_full.width = cloud_full.size();
    
    //*ptr = pl_full;
    
    //Mid360Handler(msg);
    FullCloudPtr ptr_ndt(new FullPointCloudType());
    *ptr_ndt = pl_full;

    if (ptr_ndt->empty()) {
            return;
    }

    lidar_buffer_ndt.push_back(ptr_ndt);

    //lid_raw_data_buffer.push_back(ptr);
    lid_header_time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();

    mtx_buffer.unlock();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( pl_full, output );
    
    output.header.frame_id = "map";
    output.header.stamp = msg->header.stamp;
    // std::cout<<"360 frame id : "<< output.header.frame_id << endl;
    pub_pcl.publish( output );
    
}

// void LIO::Mid360Handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg) {
//     cloud_out_.clear();
//     cloud_full_.clear();
//     int plsize = msg->point_num;

//     cloud_out_.reserve(plsize);
//     cloud_full_.resize(plsize);

//     std::vector<bool> is_valid_pt(plsize, false);
//     std::vector<uint> index(plsize - 1);
//     for (uint i = 0; i < plsize - 1; ++i) {
//         index[i] = i + 1;  // 从1开始
//     }
//     // std::execution::par_unseq, 
//     std::for_each(index.begin(), index.end(), [&](const uint &i) {
//         if ((msg->points[i].line < num_scans_) &&
//             ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
//                 if (msg->points[i].x < -0.5 || msg->points[i].x > 0.5 || 
//                     msg->points[i].y < -1.5  || msg->points[i].y > 0.3 || 
//                     msg->points[i].z < 0.1 || msg->points[i].z > -1.7) // filter out points with invalid coordinates
//                     {
//                         if (i % point_filter_num_ == 0) {
//                                 cloud_full_[i].x = msg->points[i].x;
//                                 cloud_full_[i].y = msg->points[i].y;
//                                 cloud_full_[i].z = msg->points[i].z;
//                                 cloud_full_[i].intensity = msg->points[i].reflectivity;
//                                 cloud_full_[i].offset_time = msg->points[i].offset_time / float(1000000);

//                                 if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
//                                     (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
//                                     (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7)) {
//                                     is_valid_pt[i] = true;
//                                 }
//                             }
                        
//                     }
            
//         }
//     });

//     for (uint i = 1; i < plsize; i++) {
//         if (is_valid_pt[i]) {
//             cloud_out_.points.push_back(cloud_full_[i]);
//         }
//     }
// }


//处理的imu一个点
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
    //imu_buffer.push_back(msg);
    imu_buffer.push_back(imu);
    mtx_buffer.unlock();

    return;
}

//9.80665

void LIO::image_callback( const sensor_msgs::ImageConstPtr &msg ){


    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
    double msg_header_time = msg->header.stamp.toSec(); //+ img_time_offset;
    if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
    // ROS_INFO("Get image, its header time: %.6f", msg_header_time);
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
    cv_ptr_compressed->image.release();
    img_buffer.push_back(image_cur);
    img_time_buffer.push_back(img_time_correct);

    last_timestamp_img = img_time_correct;
    mtx_buffer.unlock();

    // cv_bridge::CvImage out_msg;
    // out_msg.header.stamp = msg->header.stamp;               // Same timestamp and tf frame as input image
    // out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    // out_msg.image = image_get;                                   // Your cv::Mat
    // out_msg.header.frame_id = "map";

    // // std::cout<<"color frame id : "<< out_msg.header.frame_id << endl;
    // pub_img.publish( out_msg );
}

bool LIO::sync_packages(LidarMeasureGroup &meas)
{
    if(lidar_buffer_ndt.empty() && lidar_en) return false;
    if(img_buffer.empty() && img_en) return false;

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
        Eigen::Vector3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;
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
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix().cast<float>());
    scan_undistort_ = scan_undistort_trans;
    
    current_scan_ = ConvertToCloud<FullPointType>(scan_undistort_);

    // voxel 之
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(0.5, 0.5, 0.5);
    voxel.setInputCloud(current_scan_);

    CloudPtr current_scan_filter(new PointCloudType);
    voxel.filter(*current_scan_filter);

    if (flg_first_scan_) {
        ndt_.AddCloud(current_scan_);
        flg_first_scan_ = false;

        return;
    }

    // 后续的scan，使用NDT配合pose进行更新
    // LOG(INFO) << "=== frame " << frame_num_;

    ndt_.SetSource(current_scan_filter);
    ieskf_.UpdateUsingCustomObserve([this](const SE3 &input_pose, Mat18d &HTVH, Vec18d &HTVr) {
        ndt_.ComputeResidualAndJacobians(input_pose, HTVH, HTVr);
    });

    auto current_nav_state = ieskf_.GetNominalState();

    // 若运动了一定范围，则把点云放入地图中
    SE3 current_pose = ieskf_.GetNominalSE3();
    SE3 delta_pose = last_pose_.inverse() * current_pose;

    // CloudPtr current_scan_world(new PointCloudType);
    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr current_scan_world{new pcl::PointXYZINormal};
    pcl::transformPointCloud(*current_scan_filter, *current_scan_world, current_pose.matrix());
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_, current_pose.matrix());

    sensor_msgs::PointCloud2 output1;
    pcl::toROSMsg( *scan_undistort_, output1 );
    output1.header.frame_id = "map";
    // output.header.stamp = ros::Time::fromSec(LidarMeasures.lidar_frame_end_time);
    
    // std::cout<<"360 frame id : "<< output.header.frame_id << endl;
    pub_pcl_un.publish( output1 );

    *pcl_wait_save += *current_scan_world;
    // pcl::concatenate(*pcl_wait_save, *current_scan_world, *pcl_wait_save);

    if (delta_pose.translation().norm() > 1.0 || delta_pose.so3().log().norm() > deg2rad(10.0)) {
        // 将地图合入NDT中
        
        ndt_.AddCloud(current_scan_world);
        last_pose_ = current_pose;
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

void LIO::savePCD()
{
    if (pcl_wait_save->empty()) return;
    pcl::PCDWriter pcd_writer;
    std::string raw_points_dir = "/home/yangj/Robot/code/lio_project_wk/src/lio_project/Log/PCD/all_raw_points.pcd";
    
    if (pcl_wait_save->size() > 0)
    {
        pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save);
        // ROS_INFO("Raw point cloud data saved to: %s, with point count: %ld" ,
        //     raw_points_dir.c_str() , pcl_wait_save->points.size() );
        std::cout << "Raw point cloud data saved to: " << raw_points_dir << ", with point count: " << pcl_wait_save->points.size() << std::endl;
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
    SE3 current_pose = ieskf_.GetNominalSE3();
    Eigen::Vector3d position = current_pose.matrix().block<3,1>(0,3);
    out.position.x = position.x();
    out.position.y = position.y();
    out.position.z = position.z();

    Eigen::Matrix3d rotation_matrix = current_pose.matrix().block<3,3>(0,0);
    Eigen::Quaterniond quaternion(rotation_matrix);
    quaternion.normalize();  // 规范化四元数（确保是单位四元数）

    out.orientation.x = quaternion.x();
    out.orientation.y = quaternion.y();
    out.orientation.z = quaternion.y();
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
    if(lidar_buffer_ndt.empty() && lidar_en) return;
    if(img_buffer.empty() && img_en) return;

    if (pcl_wait_save->empty() || (pcl_wait_save == nullptr)) 
    {
        std::cout << "[ VIO ] No point!!!" << std::endl;
        return;
    }

    initcamera();
    handleVIO();

}

void LIO::initcamera()
{
    return;
}

void LIO::handleVIO()
{
    if (image_get.empty()) return;
    
    // cv::Mat left_img = cv::imread(left_file, 0);   
    // cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    // cv::RNG rng;
    // int nPoints = 2000;
    // int boarder = 20;
    // VecVector2d pixels_ref;
    // vector<double> depth_ref;

    // // generate pixels in ref and load depth data
    // for (int i = 0; i < nPoints; i++) {
    //     int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
    //     int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
    //     int disparity = disparity_img.at<uchar>(y, x);
    //     double depth = fx * baseline / disparity; // you know this is disparity to depth
    //     depth_ref.push_back(depth);
    //     pixels_ref.push_back(Eigen::Vector2d(x, y));
    // }

    // // estimates 01~05.png's pose using this information
    // Sophus::SE3 T_cur_ref;

    // for (int i = 1; i < 6; i++) {  // 1~10
    //     cv::Mat img = cv::imread((fmt_others % i).str(), 0);
    //     // try single layer by uncomment this line
    //     DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    //     // DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    // }
    // return 0;

    
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
            ROS_INFO("--------------------------------------------1");
            // // std::cout <<"imu: " << LidarMeasures.measures.back().imu.back()->header.stamp.toSec() << std::endl;
            // ROS_INFO("[LidarMeasures] LidarMeasures measures.size: %ld",LidarMeasures.measures.size());
            // ROS_INFO("[LidarMeasures] LidarMeasures lidar_frame_end_time: %.6f",LidarMeasures.lidar_frame_end_time);
            // std::cout << LidarMeasures.measures.front().imu.back()->timestamp_ << std::endl;
            // ROS_INFO("[LidarMeasures] LidarMeasures imu lio time: %.6f",LidarMeasures.measures.front().lio_time);
            // LidarMeasures.measures.pop_front();
            rate.sleep();
            continue;
        }
        ROS_INFO("--------------------------------------------2.2");
        handleFirstFrame();
        ROS_INFO("--------------------------------------------2");
        auto time2 = std::chrono::high_resolution_clock::now(); 

        ProcessIMU();
        ROS_INFO("--------------------------------------------3");
        
        ProcessLidar();

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
    ros::spin();

    
}