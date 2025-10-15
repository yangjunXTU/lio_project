/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-08-12 02:03:20
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-10-15 05:56:52
 * @FilePath: /lio_project_wk/src/lio_project/src/lio_node.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <execution>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "utils/iekf.h"
#include "utils/eigen_types.h"
#include "utils/static_imu_init.h"
#include "utils/math_utils.h"
#include "utils/point_types.h"
#include "utils/ndt_inc.h"
#include "utils/lidar_utils.h"


using namespace cv;
using namespace std;
using namespace sad;
#define IS_VALID( a ) ( ( abs( a ) > 1e8 ) ? true : false )
#define ANSI_COLOR_BLUE_BOLD "\x1b[1;34m"


class LIO
{
private:
    /* data */
    IESKFD ieskf_;
    std::vector<NavStated> imu_states_;
    StaticIMUInit imu_init_;

    SE3 TIL_;  // Lidar与IMU之间外参  L2I
    SE3 TLC_;  // camera与lidar之间的外参 C2L
    FullCloudPtr scan_undistort_;
    
    
    int num_scans_ = 4;                          // 扫描线数mid360
    int point_filter_num_ = 1;                   // 跳点
    // FullCloudPtr scan_undistort_trans(new FullPointCloudType);
    /// NDT数据
    CloudPtr current_scan_ = nullptr;
    CloudPtr current_scan_w = nullptr;
    CloudPtr pcl_wait_save{new PointCloudType};
    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_wait_save{new pcl::PointXYZINormal};
    bool flg_first_scan_ = true;
    int frame_num_ = 0;
    
    IncNdt3d ndt_;
    SE3 last_pose_;
    
    
    void Undistort();
    void Align();
    //void Mid360Handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg);

public:
    std::mutex mtx_buffer, mtx_buffer_imu_prop;
    //deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;

    deque<FullCloudPtr> lidar_buffer_ndt;
    deque<double> lid_header_time_buffer;
    
    //deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
    deque<IMUPtr> imu_buffer;

    double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
    bool lidar_pushed = false;
    bool is_first_frame = false;
    double _first_lidar_time = 0.0;

    bool imu_need_init_ = true;
    
    std::deque< sensor_msgs::CompressedImageConstPtr > g_received_compressed_img_msg;
    ros::NodeHandle nh;
    ros::Subscriber sub_depth_img_compLz4,sub_depth_img_comp, sub_img_comp, sub_pcl, sub_imu,sub_depth_img, sub_img,sub_camera_odom,sub_apriltag;
    ros::Publisher pub_depth_img_comp, pub_img_comp,pub_depth_img, pub_img,pub_img_comp_info;
    ros::Publisher pub_pcl,pub_pcl_un,pub_pcl_ndt,pub_camera_odom,pub_path;
    ros::Publisher pubOdomAftMapped,pubPath,mavros_pose_publisher;
    // nav_msgs::Path path;

    double  img_rec_time;
    cv::Mat image_get;
    std::string LiDAR_pointcloud_topic, IMU_topic, IMAGE_depth_compressed, IMAGE_color_compressed,IMAGE_depth_compressedLz4;
    std::string IMAGE_color,IMAGE_depth,CAMERA_odom;

    pcl::PointCloud<pcl::PointXYZI> output_cloud;

    LidarMeasureGroup LidarMeasures;

    nav_msgs::Path path;
    nav_msgs::Odometry odomAftMapped;
    geometry_msgs::Quaternion geoQuat;
    geometry_msgs::PoseStamped msg_body_pose;

    vector<double> ext_t;
    vector<double> ext_r;
    vector<double> cameraextrinT;
    vector<double> cameraextrinR;

    //imu参数配置
    double init_time_seconds = 5.0;     // 静止时间 10
    int init_imu_queue_max_size = 600;  // 初始化IMU队列最大长度 2000
    int static_odom_pulse = 5;           // 静止时轮速计输出噪声
    double max_static_gyro_var = 0.5;     // 静态下陀螺测量方差
    double max_static_acce_var = 0.05;    // 静态下加计测量方差
    double gravity_norm = 9.81;          // 重力大小
    bool use_speed_for_static_checking = false;  // 是否使用odom来判断车辆静止（部分数据集没有odom选项）
    
    //ndt参数配置
    int max_iteration = 4;        // 最大迭代次数
    double voxel_size = 1.0;      // 体素大小
    double inv_voxel_size = 1.0;  // 体素大小之逆
    int min_effective_pts  = 10;   // 最近邻点数阈值
    int min_pts_in_voxel = 5;     // 每个栅格中最小点数
    int max_pts_in_voxel  = 50;    // 每个栅格中最大点数
    double eps  = 1e-3;            // 收敛判定条件
    double res_outlier_th  = 5.0;  // 异常值拒绝阈值
    int  capacity = 100000;     // 缓存的体素数量
    

    //eskf参数配置
    int num_iterations = 3;  // 迭代次数
    double quit_eps = 1e-3;  // 终止迭代的dx大小
    double imu_dt = 0.01;         // IMU测量间隔
    double gyro_var = 1e-5;       // 陀螺测量标准差
    double acce_var = 1e-2;       // 加计测量标准差
    double bias_gyro_var = 1e-6;  // 陀螺零偏游走标准差
    double bias_acce_var = 1e-4;  // 加计零偏游走标准差
    bool update_bias_gyro = true;  // 是否更新bias
    bool update_bias_acce = true;  // 是否更新bias




    void image_callback( const sensor_msgs::ImageConstPtr &msg );
    void feat_points_cbk( const livox_ros_driver2::CustomMsg::ConstPtr &msg  );
    void imu_cbk(const sensor_msgs::ImuConstPtr &msg_in);
    void run();
    bool sync_packages(LidarMeasureGroup &meas);
    void handleFirstFrame();
    //IMUPtr imu2IMU(const sensor_msgs::ImuConstPtr &mg_in);
    void ProcessIMU();
    void TryInitIMU();
    void ProcessLidar();
    void ProcessCamera();
    void initcamera();
    void handleVIO();
    void savePCD();
    void publish_odometry(const ros::Publisher &pubOdomAftMapped);
    template <typename T> void set_posestamp(T &out);
    void publish_path(const ros::Publisher pubPath);
    void publish_mavros(const ros::Publisher &mavros_pose_publisher);

    LIO(/* args */);
    ~LIO();
};

