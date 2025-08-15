/*
 * @Auther: yangj
 * @Description:  
 * @Date: 2025-06-19 09:17:49
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-08-15 01:02:46
 */
#include"lio_node.h"
#include"utils/eigen_types.h"

LIO::LIO(/* args */)
{
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
    pub_camera_odom = nh.advertise<nav_msgs::Odometry>("/pub_camera_odom",1000);
    pub_path = nh.advertise<nav_msgs::Path>("/pub_apriltag_path",1000);

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
    imu_states_.clear();
    imu_states_.emplace_back(ieskf_.GetNominalState());

    for (auto &imu : LidarMeasures.measures.back().imu2)
    {
        ieskf_.Predict(*imu);
        imu_states_.emplace_back(ieskf_.GetNominalState());
    }
    ROS_INFO("imu_states_: x=%.3f,y=%.3f,z=%.3f", imu_states_.back().p_[0],imu_states_.back().p_[1],imu_states_.back().p_[2]);
}

void LIO::TryInitIMU()
{

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
            ROS_INFO("--------------------------------------------");
            // std::cout <<"imu: " << LidarMeasures.measures.back().imu.back()->header.stamp.toSec() << std::endl;
            ROS_INFO("[LidarMeasures] LidarMeasures measures.size: %ld",LidarMeasures.measures.size());
            ROS_INFO("[LidarMeasures] LidarMeasures lidar_frame_end_time: %.6f",LidarMeasures.lidar_frame_end_time);
            // std::cout << LidarMeasures.measures.front().imu2.back()->timestamp_ << std::endl;
            // ROS_INFO("[LidarMeasures] LidarMeasures imu lio time: %.6f",LidarMeasures.measures.front().lio_time);
            // LidarMeasures.measures.pop_front();
            rate.sleep();
            continue;
        }
        handleFirstFrame();
        ProcessIMU();

        
    }

    ros::spin();
}