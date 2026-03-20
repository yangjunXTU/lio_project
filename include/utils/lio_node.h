/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-08-12 02:03:20
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-10-29 02:10:37
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
#include "utils/frame.h"
#include "vio.h"

using namespace cv;
using namespace std;
using namespace sad;
#define IS_VALID( a ) ( ( abs( a ) > 1e8 ) ? true : false )
#define ANSI_COLOR_BLUE_BOLD "\x1b[1;34m"

enum SLAM_MODE
{
  ONLY_LO = 0,
  ONLY_LIO = 1,
  LIVO = 2
};

struct SemanticConfig {
    bool enabled = true;
    bool publish_map_point_rgb = true;
    bool publish_map_semantic = true;
    bool highlight_vio_points = true;
    double min_project_depth = 1.0;
    double max_project_depth = 30.0;
    double color_fusion_alpha = 0.7;
    int default_r = 180;
    int default_g = 180;
    int default_b = 180;
    int vio_r = 255;
    int vio_g = 64;
    int vio_b = 64;
    bool display_voxel_enabled = true;
    double display_voxel_size = 0.2;
    int max_display_points = 150000;
};

struct PcdSaveConfig {
    bool save_lidar = true;
    bool save_rgb = false;
    std::string output_dir = "Log/PCD";
    std::string raw_filename = "all_raw_points.pcd";
    std::string rgb_filename = "all_rgb_points.pcd";
    bool create_dir_if_missing = true;
};

struct DebugVisConfig {
    bool publish_img_with_point = true;
    int max_points = 3000;
    int point_radius = 2;
};


class LIO
{
private:
    IESKFD ieskf_;
    std::vector<NavStated> imu_states_;
    NavStated nav_state_w_i_;
    StaticIMUInit imu_init_;

    SE3 T_i_l;  // Lidar与IMU之间外参  L -> I
    SE3 T_l_c;  // camera与lidar之间的外参  C -> L
    FullCloudPtr scan_undistort_;
    
    int num_scans_ = 4;                          // 扫描线数mid360
    int point_filter_num_ = 1;                   // 跳点
    CloudPtr current_scan_ = nullptr;
    CloudPtr pcl_wait_save{new PointCloudType};
    CloudPtr current_scan_world{new PointCloudType};
    UiCloudPtr map_point_rgb_{new UiPointCloudType};
    UiCloudPtr map_semantic_{new UiPointCloudType};
    UiCloudPtr rgb_wait_save_{new UiPointCloudType};
    cv::Mat last_rgb_img_;
    double last_rgb_stamp_ = -1.0;
    bool flg_first_scan_lio = true;
    
    int frame_num_ = 0;
    
    IncNdt3d ndt_;
    SE3 last_keyframe_T_w_i_;
    
    
    void Undistort();
    void Align();

public:
    std::mutex mtx_buffer, mtx_buffer_imu_prop;

    deque<FullCloudPtr> lidar_buffer_ndt;
    deque<double> lid_header_time_buffer;
    
    deque<IMUPtr> imu_buffer;

    deque<cv::Mat> img_buffer;
    deque<double> img_time_buffer;

    double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
    bool lidar_pushed = false;
    bool is_first_frame = false;
    double _first_lidar_time = 0.0;

    int img_en = 1, lidar_en = 1;
    bool visual_enabled_ = true;
    SLAM_MODE slam_mode_;

    VIOPtr vio_manager;
    CameraConfig camera_config_;
    VioConfig vio_config_;
    SemanticConfig semantic_config_;
    PcdSaveConfig pcd_save_config_;
    DebugVisConfig debug_vis_config_;

    bool imu_need_init_ = true;
    
    ros::NodeHandle nh;
    ros::Subscriber sub_pcl, sub_imu, sub_img;
    ros::Publisher pub_img;
    ros::Publisher pub_pcl,pub_pcl_un,pub_pcl_ndt;
    ros::Publisher pub_img_with_point;
    ros::Publisher pub_map_point_rgb_, pub_map_semantic_;
    ros::Publisher pubOdomAftMapped,pubPath,mavros_pose_publisher;

    std::string LiDAR_pointcloud_topic, IMU_topic, IMAGE_color;

    LidarMeasureGroup LidarMeasures;

    nav_msgs::Path path;
    nav_msgs::Odometry odomAftMapped;
    geometry_msgs::PoseStamped msg_body_pose;

    double init_time_seconds = 5.0;     // 静止时间 10
    int init_imu_queue_max_size = 600;  // 初始化IMU队列最大长度 2000
    int static_odom_pulse = 5;           // 静止时轮速计输出噪声
    double max_static_gyro_var = 0.5;     // 静态下陀螺测量方差
    double max_static_acce_var = 0.05;    // 静态下加计测量方差
    double gravity_norm = 9.81;          // 重力大小
    bool use_speed_for_static_checking = false;  // 是否使用odom来判断车辆静止（部分数据集没有odom选项）
    
    int max_iteration = 4;        // 最大迭代次数
    double voxel_size = 1.0;      // 体素大小
    double inv_voxel_size = 1.0;  // 体素大小之逆
    int min_effective_pts  = 10;   // 最近邻点数阈值
    int min_pts_in_voxel = 5;     // 每个栅格中最小点数
    int max_pts_in_voxel  = 50;    // 每个栅格中最大点数
    double eps  = 1e-3;            // 收敛判定条件
    double res_outlier_th  = 5.0;  // 异常值拒绝阈值
    int  capacity = 100000;     // 缓存的体素数量
    

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
    void ProcessIMU();
    void TryInitIMU();
    void ProcessLidar();
    void ProcessCamera();
    void initcamera();
    void handleVIO();
    void BuildMapPointRGB(const CloudPtr& scan_world, const SE3& T_w_i);
    void BuildSemanticHighlightCloud(const FramePtr& frame_ref);
    bool ProjectWorldPointToImage(const Eigen::Vector3d& pt_w, const SE3& T_c_w, Eigen::Vector2d& uv, double& depth) const;
    UiPointType MakeRGBA(const Eigen::Vector3d& p, int r, int g, int b, int a = 255) const;
    void DownsampleUiCloud(const UiCloudPtr& in, UiCloudPtr& out, double voxel_size, int max_points) const;
    void PublishSemanticClouds();
    void publishImageWithProjectedPoints();
    void savePCD();
    void publish_odometry(const ros::Publisher &pubOdomAftMapped);
    template <typename T> void set_posestamp(T &out);
    void publish_path(const ros::Publisher pubPath);
    void publish_mavros(const ros::Publisher &mavros_pose_publisher);

    LIO();
    ~LIO();
};
