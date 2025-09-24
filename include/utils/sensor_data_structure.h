/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-08-14 07:01:51
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-08-14 07:01:56
 * @FilePath: /lio_project_wk/src/lio_project/include/utils/imu.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by xiang on 2021/7/19.
//

#ifndef sensor_data_structure
#define sensor_data_structure

#include <memory>
#include "utils/eigen_types.h"
#include <deque>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>

using namespace std;

/// IMU 读数
struct IMU {
    IMU() = default;
    IMU(double t, const Vec3d& gyro, const Vec3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};

using IMUPtr = std::shared_ptr<IMU>;



/// 带ring, range等其他信息的全量信息点云
struct FullPointType {
    PCL_ADD_POINT4D;
    float range = 0;
    float radius = 0;
    uint8_t intensity = 0;
    uint8_t ring = 0;
    uint8_t angle = 0;
    double offset_time = 0;
    float height = 0;
    float width = 0;
    double curvature;  //曲率
    inline FullPointType() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/// 全量点云的定义
using FullPointCloudType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullPointCloudType::Ptr;



//lidar、imu多传感器数据频率对齐、切断
struct MeasureGroup
{
//   double vio_time;
  double lio_time;
  //deque<sensor_msgs::Imu::ConstPtr> imu;
  deque<IMUPtr> imu;
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
  //PointCloudXYZI::Ptr lidar;
  FullPointCloudType::Ptr lidar;
  //PointCloudXYZI::Ptr pcl_proc_cur;
  FullPointCloudType::Ptr pcl_proc_cur_ndt;
  //PointCloudXYZI::Ptr pcl_proc_next;
  deque<struct MeasureGroup> measures;
//   EKF_STATE lio_vio_flg;
  int lidar_scan_index_now;

  LidarMeasureGroup()
  {
    lidar_frame_beg_time = -0.0;
    lidar_frame_end_time = 0.0;
    last_lio_update_time = -1.0;
    // lio_vio_flg = WAIT;
    // this->lidar.reset(new PointCloudXYZI());
    // this->pcl_proc_cur.reset(new PointCloudXYZI());
    // this->pcl_proc_next.reset(new PointCloudXYZI());
    this->measures.clear();
    lidar_scan_index_now = 0;
    last_lio_update_time = -1.0;
  };
};


#endif  // sensor_data_structure