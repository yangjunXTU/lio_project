/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-08-15 02:52:53
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-08-15 03:12:17
 * @FilePath: /lio_project_wk/src/lio_project/src/static_imu_init.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by xiang on 2021/11/11.
//

#ifndef SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H
#define SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H

#include "utils/eigen_types.h"
#include "utils/imu.h"
#include <deque>
#include <numeric>
#include <ros/ros.h>
#include<iostream>


/**
 * IMU水平静止状态下初始化器
 * 使用方法：调用AddIMU, AddOdom添加数据，使用InitSuccess获取初始化是否成功
 * 成功后，使用各Get函数获取内部参数
 *
 * 初始化器在每次调用AddIMU时尝试对系统进行初始化。在有odom的场合，初始化要求odom轮速读数接近零；没有时，假设车辆初期静止。
 * 初始化器收集一段时间内的IMU读数，按照书本3.5.4节估计初始零偏和噪声参数，提供给ESKF或者其他滤波器
 */
class StaticIMUInit {
   public:
    struct Options {
        Options() {}
        double init_time_seconds_ = 10.0;     // 静止时间
        int init_imu_queue_max_size_ = 2000;  // 初始化IMU队列最大长度
        int static_odom_pulse_ = 5;           // 静止时轮速计输出噪声
        double max_static_gyro_var = 0.5;     // 静态下陀螺测量方差
        double max_static_acce_var = 0.05;    // 静态下加计测量方差
        double gravity_norm_ = 9.81;          // 重力大小
        bool use_speed_for_static_checking_ = true;  // 是否使用odom来判断车辆静止（部分数据集没有odom选项）
    };

    /// 构造函数
    StaticIMUInit(Options options = Options()) : options_(options) {}

    /// 添加IMU数据
    bool AddIMU(const IMU& imu);
    /// 添加轮速数据
    // bool AddOdom(const Odom& odom);

    /// 判定初始化是否成功
    bool InitSuccess() const { return init_success_; }

    /// 获取各Cov, bias, gravity
    Vec3d GetCovGyro() const { return cov_gyro_; }
    Vec3d GetCovAcce() const { return cov_acce_; }
    Vec3d GetInitBg() const { return init_bg_; }
    Vec3d GetInitBa() const { return init_ba_; }
    Vec3d GetGravity() const { return gravity_; }

   private:
    /// 尝试对系统初始化
    bool TryInit();

    Options options_;                 // 选项信息
    bool init_success_ = false;       // 初始化是否成功
    Vec3d cov_gyro_ = Vec3d::Zero();  // 陀螺测量噪声协方差（初始化时评估）
    Vec3d cov_acce_ = Vec3d::Zero();  // 加计测量噪声协方差（初始化时评估）
    Vec3d init_bg_ = Vec3d::Zero();   // 陀螺初始零偏
    Vec3d init_ba_ = Vec3d::Zero();   // 加计初始零偏
    Vec3d gravity_ = Vec3d::Zero();   // 重力
    bool is_static_ = false;          // 标志车辆是否静止
    std::deque<IMU> init_imu_deque_;  // 初始化用的数据
    double current_time_ = 0.0;       // 当前时间
    double init_start_time_ = 0.0;    // 静止的初始时间
};

#endif  // SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H

/**
 * 计算一个容器内数据的均值与对角形式协方差
 * @tparam C    容器类型
 * @tparam D    结果类型
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个D类型
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
    // clang-format on
}