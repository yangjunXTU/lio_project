/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-08-13 02:24:16
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-08-20 07:01:16
 * @FilePath: /lio_project_wk/src/lio_project/src/iekf.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

 #include<iostream>
 #include"utils/eigen_types.h"
 #include"utils/nav_state.h"
 #include"utils/imu.h"
 #include"utils/math_utils.h"
 
#ifndef H_IEKF_LIO_HPP
#define H_IEKF_LIO_HPP

// 常量定义
// constexpr double kDEG2RAD = M_PI / 180.0;  // deg->rad
// constexpr double kRAD2DEG = 180.0 / M_PI;  // rad -> deg
// constexpr double G_m_s2 = 9.81;            // 重力大小

template <typename S>
class IESKF
{
    public:
        using SO3 = Sophus::SO3;                      // 旋转变量类型
        using VecT = Eigen::Matrix<S, 3 , 1>;            // 向量类型
        using Vec18T = Eigen::Matrix<S, 18 , 1>;         // 18维向量类型
        using Mat3T = Eigen::Matrix<S, 3 , 3>;           // 3x3矩阵类型
        using Mat18T = Eigen::Matrix<S, 18 , 18>;        // 18维方差类型
        using MotionNoiseT = Eigen::Matrix<S, 18, 18>;   // 运动噪声类型
        using OdomNoiseT = Eigen::Matrix<S, 3, 3>;       // 里程计噪声类型
        // using GnssNoiseT = Eigen::Matrix<S, 6, 6>;     // GNSS噪声类型
        using NavStateT = NavState<S>;                  // 整体名义状态变量类型
        
        struct Options {
            Options() = default;
            /// IEKF配置
            int num_iterations_ = 3;  // 迭代次数
            double quit_eps_ = 1e-3;  // 终止迭代的dx大小

            /// IMU 测量与零偏参数
            double imu_dt_ = 0.01;         // IMU测量间隔
            double gyro_var_ = 1e-5;       // 陀螺测量标准差
            double acce_var_ = 1e-2;       // 加计测量标准差
            double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差
            double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差

            /// RTK 观测参数
            // double gnss_pos_noise_ = 0.1;                   // GNSS位置噪声
            // double gnss_height_noise_ = 0.1;                // GNSS高度噪声
            // double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  // GNSS旋转噪声

            /// 其他配置
            bool update_bias_gyro_ = true;  // 是否更新bias
            bool update_bias_acce_ = true;  // 是否更新bias
        };


        IESKF(Options option = Options()) : options_(option) { BuildNoise(option); }
        // IESKF(/* args */);
        IESKF(Options options, const VecT& init_bg, const VecT& init_ba, const VecT& gravity = VecT(0, 0, -9.8))
            : options_(options) {
            BuildNoise(options);
            bg_ = init_bg;
            ba_ = init_ba;
            g_ = gravity;
        }

        /// 设置初始条件
        void SetInitialConditions(Options options, const VecT& init_bg, const VecT& init_ba,
                                const VecT& gravity = VecT(0, 0, -9.8)) {
            BuildNoise(options);
            options_ = options;
            bg_ = init_bg;
            ba_ = init_ba;
            g_ = gravity;

            cov_ = 1e-4 * Mat18T::Identity();
            cov_.template block<3, 3>(6, 6) = 0.1 * kDEG2RAD * Mat3T::Identity();
        }

        NavStateT GetNominalState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }

        // ~IESKF();
        bool Predict(const IMU& imu);

        using CustomObsFunc = std::function<void(const SE3& input_pose, Eigen::Matrix<S, 18, 18>& HT_Vinv_H,
                                             Eigen::Matrix<S, 18, 1>& HT_Vinv_r)>;

        /// 使用自定义观测函数更新滤波器
        bool UpdateUsingCustomObserve(CustomObsFunc obs);

        /// 全量状态
        // NavStateT GetNominalState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }

        /// SE3 状态
        SE3 GetNominalSE3() const { return SE3(R_, p_); }

        void SetX(const NavStated& x) {
            current_time_ = x.timestamp_;
            R_ = x.R_;
            p_ = x.p_;
            v_ = x.v_;
            bg_ = x.bg_;
            ba_ = x.ba_;
        }

        void SetCov(const Mat18T& cov) { cov_ = cov; }
        Vec3d GetGravity() const { return g_; }

    private:

        void BuildNoise(const Options& options) {
            double ev = options.acce_var_;
            double et = options.gyro_var_;
            double eg = options.bias_gyro_var_;
            double ea = options.bias_acce_var_;

            double ev2 = ev;  // * ev;
            double et2 = et;  // * et;
            double eg2 = eg;  // * eg;
            double ea2 = ea;  // * ea;

            // set Q
            Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

            // double gp2 = options.gnss_pos_noise_ * options.gnss_pos_noise_;
            // double gh2 = options.gnss_height_noise_ * options.gnss_height_noise_;
            // double ga2 = options.gnss_ang_noise_ * options.gnss_ang_noise_;
            // gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
        }

        /// 更新名义状态变量
        void Update() {
            p_ += dx_.template block<3, 1>(0, 0);
            v_ += dx_.template block<3, 1>(3, 0);
            R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

            if (options_.update_bias_gyro_) {
                bg_ += dx_.template block<3, 1>(9, 0);
            }

            if (options_.update_bias_acce_) {
                ba_ += dx_.template block<3, 1>(12, 0);
            }
            g_ += dx_.template block<3, 1>(15, 0);
        }

        double current_time_ = 0.0;
        
        // nominal state
        SO3 R_;
        VecT p_ = VecT::Zero();
        VecT v_ = VecT::Zero();
        VecT bg_ = VecT::Zero();
        VecT ba_ = VecT::Zero();
        VecT g_{0, 0, -9.8};

        // error state
        Vec18T dx_ = Vec18T::Zero();

        // covariance
        Mat18T cov_ = Mat18T::Identity();

        // noise
        MotionNoiseT Q_ = MotionNoiseT::Zero();
        // GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();

        Options options_;

};

using IESKFD = IESKF<double>;
using IESKFF = IESKF<float>;

template <typename S>
bool IESKF<S>::Predict(const IMU& imu){
    assert(imu.timestamp_ >= current_time_);

    double dt = imu.timestamp_ - current_time_;
    if (dt > (5 * options_.imu_dt_) || dt < 0) {
        std::cout << "skip this imu because dt_ = " << dt;
        current_time_ = imu.timestamp_;
        return false;
    }

    VecT new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
    VecT new_v = v_ + R_ * (imu.acce_ - ba_) * dt + g_ * dt;
    SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

    R_ = new_R;
    v_ = new_v;
    p_ = new_p;

    Mat18T F = Mat18T::Identity();
    F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu.acce_ - ba_) * dt;
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;
    F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;
    F.template block<3, 3>(6, 6) = SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();
    F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;
    
    cov_ = F * cov_ * F.transpose() + Q_;
    current_time_ = imu.timestamp_;
    return true;
}

// TODO: update function
// template <typename S>
// bool IESKF<S>::update(const IMU& imu){

// }

template <typename S>
bool IESKF<S>::UpdateUsingCustomObserve(IESKF::CustomObsFunc obs) {
    // H阵由用户给定

    SO3 start_R = R_;
    Eigen::Matrix<S, 18, 1> HTVr;
    Eigen::Matrix<S, 18, 18> HTVH;
    Eigen::Matrix<S, 18, Eigen::Dynamic> K;
    Mat18T Pk, Qk;

    for (int iter = 0; iter < options_.num_iterations_; ++iter) {
        // 调用obs function
        obs(GetNominalSE3(), HTVH, HTVr);

        // 投影P
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat((R_.inverse() * start_R).log());
        Pk = J * cov_ * J.transpose();

        // 卡尔曼更新
        Qk = (Pk.inverse() + HTVH).inverse();  // 这个记作中间变量，最后更新时可以用
        dx_ = Qk * HTVr;
        // LOG(INFO) << "iter " << iter << " dx = " << dx_.transpose() << ", dxn: " << dx_.norm();

        // dx合入名义变量
        Update();

        if (dx_.norm() < options_.quit_eps_) {
            break;
        }
    }

    // update P
    cov_ = (Mat18T::Identity() - Qk * HTVH) * Pk;

    // project P
    Mat18T J = Mat18T::Identity();
    Vec3d dtheta = (R_.inverse() * start_R).log();
    J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dtheta);
    cov_ = J * cov_ * J.inverse();

    dx_.setZero();
    return true;
}

#endif