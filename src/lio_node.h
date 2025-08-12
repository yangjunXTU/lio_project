#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <unistd.h>
#include <deque>
#include <fstream>
#include <livox_ros_driver2/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <roslz4/lz4s.h>

#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<sensor_msgs/Image.h>
#include<boost/bind.hpp>
#include<functional>

#include<geometry_msgs/PoseWithCovariance.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/CameraInfo.h>
#include<sensor_msgs/Imu.h>


using namespace cv;
using namespace std;
#define IS_VALID( a ) ( ( abs( a ) > 1e8 ) ? true : false )
#define ANSI_COLOR_BLUE_BOLD "\x1b[1;34m"


class LIO
{
private:
    /* data */
public:

    std::deque< sensor_msgs::CompressedImageConstPtr > g_received_compressed_img_msg;
    ros::NodeHandle nh;
    ros::Subscriber sub_depth_img_compLz4,sub_depth_img_comp, sub_img_comp, sub_pcl, sub_imu,sub_depth_img, sub_img,sub_camera_odom,sub_apriltag;
    ros::Publisher pub_depth_img_comp, pub_img_comp,pub_depth_img, pub_img,pub_img_comp_info;
    ros::Publisher pub_pcl,pub_camera_odom,pub_path;
    nav_msgs::Path path;

    double  img_rec_time;
    cv::Mat image_get;
    std::string LiDAR_pointcloud_topic, IMU_topic, IMAGE_depth_compressed, IMAGE_color_compressed,IMAGE_depth_compressedLz4;
    std::string IMAGE_color,IMAGE_depth,CAMERA_odom;

    pcl::PointCloud<pcl::PointXYZI> output_cloud;

    void image_callback( const sensor_msgs::ImageConstPtr &msg );
    void feat_points_cbk( const livox_ros_driver2::CustomMsg::ConstPtr &msg  );
    void imu_cbk(const sensor_msgs::ImuConstPtr &msg_in);


    LIO(/* args */);
    ~LIO();
};

