/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-08-12 02:03:20
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-08-14 09:30:15
 * @FilePath: /lio_project_wk/src/lio_project/src/lio_node.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
#include "utils/types.h"
#include "iekf.h"
#include "utils/eigen_types.h"

using namespace cv;
using namespace std;
#define IS_VALID( a ) ( ( abs( a ) > 1e8 ) ? true : false )
#define ANSI_COLOR_BLUE_BOLD "\x1b[1;34m"


struct MeasureGroup
{
//   double vio_time;
  double lio_time;
  deque<sensor_msgs::Imu::ConstPtr> imu;
  deque<IMUPtr> imu2;
//   cv::Mat img;
  MeasureGroup()
  {
    // vio_time = 0.0;
    lio_time = 0.0;
  };
};

struct LidarMeasureGroup
{
  double lidar_frame_beg_time;
  double lidar_frame_end_time;
  double last_lio_update_time;
  PointCloudXYZI::Ptr lidar;
  PointCloudXYZI::Ptr pcl_proc_cur;
  PointCloudXYZI::Ptr pcl_proc_next;
  deque<struct MeasureGroup> measures;
//   EKF_STATE lio_vio_flg;
  int lidar_scan_index_now;

  LidarMeasureGroup()
  {
    lidar_frame_beg_time = -0.0;
    lidar_frame_end_time = 0.0;
    last_lio_update_time = -1.0;
    // lio_vio_flg = WAIT;
    this->lidar.reset(new PointCloudXYZI());
    this->pcl_proc_cur.reset(new PointCloudXYZI());
    this->pcl_proc_next.reset(new PointCloudXYZI());
    this->measures.clear();
    lidar_scan_index_now = 0;
    last_lio_update_time = -1.0;
  };
};

class LIO
{
private:
    /* data */
public:
    std::mutex mtx_buffer, mtx_buffer_imu_prop;
    deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
    deque<double> lid_header_time_buffer;
    deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
    double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
    bool lidar_pushed = false;
    bool is_first_frame = false;
    double _first_lidar_time = 0.0;
    
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

    LidarMeasureGroup LidarMeasures;

    void image_callback( const sensor_msgs::ImageConstPtr &msg );
    void feat_points_cbk( const livox_ros_driver2::CustomMsg::ConstPtr &msg  );
    void imu_cbk(const sensor_msgs::ImuConstPtr &msg_in);
    void run();
    bool sync_packages(LidarMeasureGroup &meas);
    void handleFirstFrame();
    IMUPtr imu2IMU(const sensor_msgs::ImuConstPtr &mg_in);
    void ProcessIMU();

    LIO(/* args */);
    ~LIO();
};

