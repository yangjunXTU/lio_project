#include "utils/ndt_inc.h"
#include "utils/lidar_utils.h"
#include "utils/math_utils.h" 
#include "timer/timer.h"

#include <glog/logging.h>
#include <execution>
#include <set>

namespace sad {

// ------------------- 增量式NDT核心实现 -------------------
void IncNdt3d::AddCloud(CloudPtr cloud_world) {
    std::set<KeyType, less_vec<3>> active_voxels;  // 记录本次点云更新的voxel索引

    for (const auto& p : cloud_world->points) {
        auto pt = ToVec3d(p);  // 点坐标
        auto key = CastToInt(Vec3d(pt * options_.inv_voxel_size_));  // 点所在的栅格索引
        auto iter = grids_.find(key);

        if (iter == grids_.end()) {
            // 如果栅格不存在，新建一个
            data_.push_front({key, {pt}});
            grids_.insert({key, data_.begin()});

            // 容量超过限制，移除最久未使用的栅格
            if (data_.size() >= options_.capacity_) {
                grids_.erase(data_.back().first);
                data_.pop_back();
            }
        } else {
            // 如果栅格存在，加入点并更新LRU缓存
            iter->second->second.AddPoint(pt);
            data_.splice(data_.begin(), data_, iter->second);  // 移到队头
            iter->second = data_.begin();
        }

        active_voxels.emplace(key);  // 记录被更新的栅格
    }

    // 并行更新这些active_voxel
    std::for_each(active_voxels.begin(), active_voxels.end(),
                  [this](const auto& key) { UpdateVoxel(grids_[key]->second); });

    flag_first_scan_ = false;
}

// ------------------- 生成邻近栅格 -------------------
void IncNdt3d::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());  // 只用中心格
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        // 当前格 + 六邻域
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),
                         KeyType(0, 1, 0), KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}

// ------------------- 更新某个栅格的均值和协方差 -------------------
void IncNdt3d::UpdateVoxel(VoxelData& v) {
    if (flag_first_scan_) {  
        // 首帧，直接用点计算均值和协方差
        if (v.pts_.size() > 1) {
            ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vec3d& p) { return p; });
            v.info_ = (v.sigma_ + Mat3d::Identity() * 1e-3).inverse();  // 防止不可逆
        } else {
            // 点太少，给一个较大的信息矩阵
            v.mu_ = v.pts_[0];
            v.info_ = Mat3d::Identity() * 1e2;
        }
        v.ndt_estimated_ = true;
        v.pts_.clear();
        return;
    }

    if (v.ndt_estimated_ && v.num_pts_ > options_.max_pts_in_voxel_) {
        return;  // 栅格点数足够，跳过
    }

    if (!v.ndt_estimated_ && v.pts_.size() > options_.min_pts_in_voxel_) {
        // 新增的voxel，开始估计
        ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vec3d& p) { return p; });
        v.info_ = (v.sigma_ + Mat3d::Identity() * 1e-3).inverse();
        v.ndt_estimated_ = true;
        v.pts_.clear();
    } else if (v.ndt_estimated_ && v.pts_.size() > options_.min_pts_in_voxel_) {
        // 已估计过，但有新点进来，进行增量更新
        Vec3d cur_mu, new_mu;
        Mat3d cur_var, new_var;
        ComputeMeanAndCov(v.pts_, cur_mu, cur_var, [this](const Vec3d& p) { return p; });
        UpdateMeanAndCov(v.num_pts_, v.pts_.size(), v.mu_, v.sigma_, cur_mu, cur_var, new_mu, new_var);

        v.mu_ = new_mu;
        v.sigma_ = new_var;
        v.num_pts_ += v.pts_.size();
        v.pts_.clear();

        // 奇异值分解，避免协方差矩阵退化
        Eigen::JacobiSVD svd(v.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vec3d lambda = svd.singularValues();
        if (lambda[1] < lambda[0] * 1e-3) lambda[1] = lambda[0] * 1e-3;
        if (lambda[2] < lambda[0] * 1e-3) lambda[2] = lambda[0] * 1e-3;

        Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
        v.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
    }
}

// ------------------- NDT对齐 -------------------
bool IncNdt3d::AlignNdt(SE3& init_pose) {
    LOG(INFO) << "aligning with inc ndt, pts: " << source_->size() << ", grids: " << grids_.size();
    assert(!grids_.empty());

    SE3 pose = init_pose;

    int num_residual_per_point = (options_.nearby_type_ == NearbyType::NEARBY6) ? 7 : 1;

    // 点索引
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) index[i] = i;

    int total_size = index.size() * num_residual_per_point;

    // 迭代优化
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        std::vector<bool> effect_pts(total_size, false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
        std::vector<Vec3d> errors(total_size);
        std::vector<Mat3d> infos(total_size);

        // 并行计算残差与雅可比
        std::for_each(index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;  // 变换到目标坐标系
            Vec3i key = CastToInt(Vec3d(qs * options_.inv_voxel_size_));

            for (int i = 0; i < nearby_grids_.size(); ++i) {
                Vec3i real_key = key + nearby_grids_[i];
                auto it = grids_.find(real_key);
                int real_idx = idx * num_residual_per_point + i;

                if (it != grids_.end() && it->second->second.ndt_estimated_) {
                    auto& v = it->second->second;
                    Vec3d e = qs - v.mu_;
                    double res = e.transpose() * v.info_ * e;

                    if (std::isnan(res) || res > options_.res_outlier_th_) {
                        effect_pts[real_idx] = false;
                        continue;
                    }

                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(q); // 对旋转
                    J.block<3, 3>(0, 3) = Mat3d::Identity();                  // 对平移

                    jacobians[real_idx] = J;
                    errors[real_idx] = e;
                    infos[real_idx] = v.info_;
                    effect_pts[real_idx] = true;
                } else {
                    effect_pts[real_idx] = false;
                }
            }
        });

        // 累加Hessian和误差
        double total_res = 0;
        int effective_num = 0;
        Mat6d H = Mat6d::Zero();
        Vec6d err = Vec6d::Zero();

        for (int idx = 0; idx < effect_pts.size(); ++idx) {
            if (!effect_pts[idx]) continue;
            total_res += errors[idx].transpose() * infos[idx] * errors[idx];
            effective_num++;
            H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
            err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
        }

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            init_pose = pose;
            return false;
        }

        // 高斯-牛顿更新
        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm()
                  << ", dx: " << dx.transpose();

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }

    init_pose = pose;
    return true;
}

// ------------------- 计算残差与雅可比 -------------------
void IncNdt3d::ComputeResidualAndJacobians(const SE3& input_pose, Mat18d& HTVH, Vec18d& HTVr) {
    assert(!grids_.empty());
    SE3 pose = input_pose;

    int num_residual_per_point = (options_.nearby_type_ == NearbyType::NEARBY6) ? 7 : 1;

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) index[i] = i;

    int total_size = index.size() * num_residual_per_point; 

    std::vector<bool> effect_pts(total_size, false);
    std::vector<Eigen::Matrix<double, 3, 18>> jacobians(total_size);
    std::vector<Vec3d> errors(total_size);
    std::vector<Mat3d> infos(total_size);

    auto time1 = std::chrono::high_resolution_clock::now(); 
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
        auto q = ToVec3d(source_->points[idx]);
        Vec3d qs = pose * q;
        Vec3i key = CastToInt(Vec3d(qs * options_.inv_voxel_size_));

        for (int i = 0; i < nearby_grids_.size(); ++i) {
            Vec3i real_key = key + nearby_grids_[i];
            auto it = grids_.find(real_key);
            int real_idx = idx * num_residual_per_point + i;

            if (it != grids_.end() && it->second->second.ndt_estimated_) {
                auto& v = it->second->second;
                Vec3d e = qs - v.mu_;
                double res = e.transpose() * v.info_ * e;

                if (std::isnan(res) || res > options_.res_outlier_th_) {
                    effect_pts[real_idx] = false;
                    continue;
                }

                Eigen::Matrix<double, 3, 18> J;
                J.setZero();
                J.block<3, 3>(0, 0) = Mat3d::Identity();                   // 对p
                J.block<3, 3>(0, 6) = -pose.so3().matrix() * SO3::hat(q);  // 对R

                jacobians[real_idx] = J;
                errors[real_idx] = e;
                infos[real_idx] = v.info_;
                effect_pts[real_idx] = true;
            } else {
                effect_pts[real_idx] = false;
            }
        }
    });

    // 累加 Hessian 与 残差
    double total_res = 0;
    int effective_num = 0;
    HTVH.setZero();
    HTVr.setZero();

    const double info_ratio = 0.01;  // 每个点的权重

    auto time2 = std::chrono::high_resolution_clock::now(); 
    for (int idx = 0; idx < effect_pts.size(); ++idx) {
        if (!effect_pts[idx]) continue;

        total_res += errors[idx].transpose() * infos[idx] * errors[idx];
        effective_num++;

        HTVH += jacobians[idx].transpose() * infos[idx] * jacobians[idx] * info_ratio;
        HTVr += -jacobians[idx].transpose() * infos[idx] * errors[idx] * info_ratio;
    }
    auto time3 = std::chrono::high_resolution_clock::now(); 

    std::chrono::duration<double> duration_time = time2 - time1;
    std::chrono::duration<double> duration_time2 = time3 - time2;

    LOG(INFO) << "effective: " << effective_num 
              << " time_J: " << duration_time.count() 
              << " sum_J: " << duration_time2.count()
              << " grid.size: " << grids_.size();
}

}  // namespace sad
