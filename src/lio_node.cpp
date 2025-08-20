/*
 * @Auther: yangj
 * @Description:  
 * @Date: 2025-06-19 09:17:49
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-08-20 07:37:04
 */
#include"lio_node.h"
#include"utils/eigen_types.h"
#include <execution>


LIO::LIO(/* args */)
{
    FullCloudPtr scan_undistort_{new FullPointCloudType()};
    
    nh.param<std::string> ("/LiDAR_pointcloud_topic",LiDAR_pointcloud_topic,std::string("/livox/lidar"));
    nh.param<std::string> ("/IMAGE_color",IMAGE_color,std::string("/camera/color/image_raw"));
    nh.param<std::string> ("/IMU_topic",IMU_topic,std::string("/livox/imu"));

    sub_img = nh.subscribe(IMAGE_color.c_str(), 1000000, &LIO::image_callback, this, ros::TransportHints().tcpNoDelay());
    sub_pcl = nh.subscribe(LiDAR_pointcloud_topic.c_str(), 2000000, &LIO::feat_points_cbk, this, ros::TransportHints().tcpNoDelay());
    sub_imu = nh.subscribe(IMU_topic.c_str(), 10, &LIO::imu_cbk, this, ros::TransportHints().tcpNoDelay());

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

    std::vector<double> ext_t = {-0.011, -0.02329, 0.04412};
    std::vector<double> ext_r ={1, 0, 0, 0, 1, 0, 0, 0, 1};

    Eigen::Vector3d lidar_T_wrt_IMU = VecFromArray(ext_t);
    Eigen::Matrix3d lidar_R_wrt_IMU = MatFromArray(ext_r);
    TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
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

    // pcl::PointCloud<pcl::PointXYZINormal> pl_full;
    PointCloudXYZI pl_full;
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

    mtx_buffer.lock();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lid_raw_data_buffer.clear();
    }

    uint plsize = msg->point_num;
    pl_full.resize( plsize );

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

        // clang-format on
            pl_full[ i ].x = msg->points[ i ].x;
            pl_full[ i ].y = msg->points[ i ].y;
            pl_full[ i ].z = msg->points[ i ].z;
            pl_full[ i ].intensity = msg->points[ i ].reflectivity;

            pl_full[ i ].curvature = msg->points[ i ].offset_time / float( 1000000 ); // use curvature as time of each laser points

        }

    }

    pl_full.height = 1;
    pl_full.width = pl_full.size();
    
    *ptr = pl_full;
    
    Mid360Handler(msg);
    FullCloudPtr ptr_ndt(new FullPointCloudType());
    *ptr_ndt = cloud_out_;

    if (ptr_ndt->empty()) {
            return;
        }

    lidar_buffer_ndt.emplace_back(ptr_ndt);

    lid_raw_data_buffer.push_back(ptr);
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

void LIO::Mid360Handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    int plsize = msg->point_num;

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<bool> is_valid_pt(plsize, false);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }
    // std::execution::par_unseq, 
    std::for_each(index.begin(), index.end(), [&](const uint &i) {
        if ((msg->points[i].line < num_scans_) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num_ == 0) {
                cloud_full_[i].x = msg->points[i].x;
                cloud_full_[i].y = msg->points[i].y;
                cloud_full_[i].z = msg->points[i].z;
                cloud_full_[i].intensity = msg->points[i].reflectivity;
                cloud_full_[i].time = msg->points[i].offset_time / float(1000000);

                if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                    (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                    (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7)) {
                    is_valid_pt[i] = true;
                }
            }
        }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }
}

void LIO::imu_cbk(const sensor_msgs::ImuConstPtr &msg_in)
{
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
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();

    return;
}


void LIO::image_callback( const sensor_msgs::ImageConstPtr &msg )
{
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
    // img_rec_time = msg->header.stamp.toSec();
    image_get = cv_ptr_compressed->image;
    cv_ptr_compressed->image.release();

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = msg->header.stamp;               // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    out_msg.image = image_get;                                   // Your cv::Mat
    out_msg.header.frame_id = "map";

    // std::cout<<"color frame id : "<< out_msg.header.frame_id << endl;

    pub_img.publish( out_msg );

}

bool LIO::sync_packages(LidarMeasureGroup &meas)
{
    if (lid_raw_data_buffer.empty()) return false;
    if (imu_buffer.empty()) return false;

    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      // 激光雷达点云指针缓存器的第一帧点云给 给雷达测量组meas.lidar
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;  // 判断点云数量

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      meas.pcl_proc_cur_ndt = lidar_buffer_ndt.front();
      lidar_pushed = true;                                                                                       // flag
    }
    // TODO: add imu data to meas
    if (last_timestamp_imu < meas.lidar_frame_end_time)
    { 
      return false;
    }

    struct MeasureGroup m;
    m.imu.clear();
    m.imu2.clear();
    m.lio_time = meas.lidar_frame_end_time;

    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time) break;
      IMUPtr imu2 = imu2IMU(imu_buffer.front());
      m.imu2.emplace_back(imu2);
    //   ROS_INFO("imu2->timestamp_: %.6f ",  imu2->timestamp_);
      m.imu.push_back(imu_buffer.front()); // 添加IMU数据
      imu_buffer.pop_front(); // 移除处理过的IMU数据
    }
    // 移除激光雷达数据和时间戳
    lid_raw_data_buffer.pop_front();
    lidar_buffer_ndt.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();

    // meas.lio_vio_flg = LIO; // 标记为LIO模式
    meas.measures.push_back(m); // 将测量数据添加到队列中
    lidar_pushed = false; // 重置标记

    return true;
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


IMUPtr LIO::imu2IMU(const sensor_msgs::ImuConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    IMUPtr imu;
    imu =std::make_shared<IMU>(msg->header.stamp.toSec(),
                                Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                Eigen::Vector3d(msg->linear_acceleration.x * 9.80665, msg->linear_acceleration.y * 9.80665,
                                    msg->linear_acceleration.z * 9.80665));
    return imu;
}

void LIO::ProcessIMU()
{
    // ROS_INFO("ProcessIMU....");
    // imu init
    if (imu_need_init_) {
        // 初始化IMU系统
        // ROS_INFO("start init---->");
        TryInitIMU();
        return;
    }

    // predict imu state
    imu_states_.clear();
    imu_states_.emplace_back(ieskf_.GetNominalState());

    for (auto &imu : LidarMeasures.measures.back().imu2)
    {
        ieskf_.Predict(*imu);
        imu_states_.emplace_back(ieskf_.GetNominalState());
    }
    ROS_INFO("imu_states_: x=%.3f,y=%.3f,z=%.3f", imu_states_.back().p_[0],imu_states_.back().p_[1],imu_states_.back().p_[2]);

    // lildar undistort
    Undistort();

    Align();
}

void LIO::TryInitIMU()
{
    for (auto imu : LidarMeasures.measures.back().imu2) {
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
    auto cloud = LidarMeasures.pcl_proc_cur_ndt;
    auto imu_state = ieskf_.GetNominalState();
    SE3 T_end = SE3(imu_state.R_, imu_state.p_);

    //std::execution::par_unseq, 并发模式 c++17以上
    std::for_each( cloud->points.begin(), cloud->points.end(),[&](auto &pt){
        SE3 Ti = T_end;
        NavStated match;
        // auto time_p = pt.curvature / double(1000);
        //* 1e-3
        PoseInterp<NavStated>(
            LidarMeasures.lidar_frame_beg_time + pt.time * 1e-3 , imu_states_, [](const NavStated &s) { return s.timestamp_; },
            [](const NavStated &s) { return s.GetSE3(); }, Ti, match);

        Eigen::Vector3d pi = ToVec3d(pt);
        Eigen::Vector3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;
        
        pt.x = p_compensate(0);
        pt.y = p_compensate(1);
        pt.z = p_compensate(2);

    });

    scan_undistort_ = cloud;
    // TODO: 需要解决scan_undistort_、cloud数据类型不匹配的问题；

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( *cloud, output );
    output.header.frame_id = "map";
    // output.header.stamp = ros::Time::fromSec(LidarMeasures.lidar_frame_end_time);
    
    // std::cout<<"360 frame id : "<< output.header.frame_id << endl;
    pub_pcl_un.publish( output );

}

void LIO::Align()
{
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
    LOG(INFO) << "=== frame " << frame_num_;

    ndt_.SetSource(current_scan_filter);
    ieskf_.UpdateUsingCustomObserve([this](const SE3 &input_pose, Mat18d &HTVH, Vec18d &HTVr) {
        ndt_.ComputeResidualAndJacobians(input_pose, HTVH, HTVr);
    });

    auto current_nav_state = ieskf_.GetNominalState();

    // 若运动了一定范围，则把点云放入地图中
    SE3 current_pose = ieskf_.GetNominalSE3();
    SE3 delta_pose = last_pose_.inverse() * current_pose;

    CloudPtr current_scan_world(new PointCloudType);
    pcl::transformPointCloud(*current_scan_filter, *current_scan_world, current_pose.matrix());

    if (delta_pose.translation().norm() > 1.0 || delta_pose.so3().log().norm() > deg2rad(10.0)) {
        // 将地图合入NDT中
        
        ndt_.AddCloud(current_scan_world);
        last_pose_ = current_pose;
    }

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

void LIO::stateEstimationAndMapping()
{
    
    return;
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
        if (!sync_packages(LidarMeasures))  
        {   
            // ROS_INFO("--------------------------------------------");
            // // std::cout <<"imu: " << LidarMeasures.measures.back().imu.back()->header.stamp.toSec() << std::endl;
            // ROS_INFO("[LidarMeasures] LidarMeasures measures.size: %ld",LidarMeasures.measures.size());
            // ROS_INFO("[LidarMeasures] LidarMeasures lidar_frame_end_time: %.6f",LidarMeasures.lidar_frame_end_time);
            // std::cout << LidarMeasures.measures.front().imu2.back()->timestamp_ << std::endl;
            // ROS_INFO("[LidarMeasures] LidarMeasures imu lio time: %.6f",LidarMeasures.measures.front().lio_time);
            // LidarMeasures.measures.pop_front();
            rate.sleep();
            continue;
        }
        handleFirstFrame();
        ProcessIMU();
        
        // stateEstimationAndMapping();
        
    }

    ros::spin();
}