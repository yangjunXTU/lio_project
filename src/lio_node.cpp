/*
 * @Auther: yangj
 * @Description:  
 * @Date: 2025-06-19 09:17:49
 * @LastEditors: yangj
 * @LastEditTime: 2025-06-19 09:56:04
 */
#include"lio_node.h"

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
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/mid360",1000);
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

    pcl::PointCloud<pcl::PointXYZINormal> pl_full;

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
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( pl_full, output );
    output.header.frame_id = "map";
    output.header.stamp = msg->header.stamp;
    // std::cout<<"360 frame id : "<< output.header.frame_id << endl;
    pub_pcl.publish( output );
    

}

void LIO::imu_cbk(const sensor_msgs::ImuConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg( new sensor_msgs::Imu( *msg_in ) );
    double timestamp = msg->header.stamp.toSec();

    


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

