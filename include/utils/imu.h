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

#ifndef MAPPING_IMU_H
#define MAPPING_IMU_H

#include <memory>
#include "utils/eigen_types.h"



/// IMU 读数
struct IMU {
    IMU() = default;
    IMU(double t, const Vec3d& gyro, const Vec3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};


using IMUPtr = std::shared_ptr<IMU>;

#endif  // MAPPING_IMU_H