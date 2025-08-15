/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-08-12 02:35:26
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-08-12 03:01:25
 * @FilePath: /lio_project_wk/src/lio_project/include/utils/types.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointXYZRGBA PointTypeRGBA;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<PointTypeRGBA> PointCloudXYZRGBA;

typedef Eigen::Vector2f V2F;
typedef Eigen::Vector2d V2D;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#define VD(a) Eigen::Matrix<double, (a), 1>
#define MF(a, b) Eigen::Matrix<float, (a), (b)>
#define VF(a) Eigen::Matrix<float, (a), 1>

struct Pose6D
{
  /*** the preintegrated Lidar states at the time of IMU measurements in a frame ***/
  double offset_time; // the offset time of IMU measurement w.r.t the first lidar point
  double acc[3];      // the preintegrated total acceleration (global frame) at the Lidar origin
  double gyr[3];      // the unbiased angular velocity (body frame) at the Lidar origin
  double vel[3];      // the preintegrated velocity (global frame) at the Lidar origin
  double pos[3];      // the preintegrated position (global frame) at the Lidar origin
  double rot[9];      // the preintegrated rotation (global frame) at the Lidar origin
};

#endif