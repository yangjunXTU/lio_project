#include "utils/icp_inc.h"
#include "utils/math_utils.h"
#include "utils/eigen_types.h"
#include <execution>
#include <vector>
#include <numeric>

namespace sad {
void Icp3d::BuildTargetKdTree() {
    kdtree_ = std::make_shared<KdTree>();
    kdtree_->BuildTree(target_);
    kdtree_->SetEnableANN();
}
bool Icp3d::ComputeResidualAndJacobiansP2P(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr) {
    LOG(INFO) << "aligning with point to point"; 
    assert(target_ != nullptr && source_ != nullptr);

    HTVH.setZero();
    HTVr.setZero();
    const double info_ratio = 0.01;

    // 对源点云生成索引
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) index[i] = i;

    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
    std::vector<Vec3d> errors(index.size());

    // 复制一份 pose 用于迭代更新
    SE3 pose_iter = pose;

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose_iter * q; 
            std::vector<int> nn;
            kdtree_->GetClosestPoint(ToPointType(qs), nn, 1);

            if (!nn.empty()) {
                Vec3d p = ToVec3d(target_->points[nn[0]]);
                double dis2 = (p - qs).squaredNorm();
                if (dis2 > options_.max_nn_distance_) {
                    effect_pts[idx] = false;
                    return;
                }

                effect_pts[idx] = true;
                Vec3d e = p - qs;

                Eigen::Matrix<double, 3, 6> J;
                J.block<3,3>(0,0) = pose_iter.so3().matrix() * SO3::hat(q);
                J.block<3,3>(0,3) = -Mat3d::Identity();

                jacobians[idx] = J;
                errors[idx] = e;
            } else {
                effect_pts[idx] = false;
            }
        });

        double total_res = 0;
        int effective_num = 0;
        auto H_and_err = std::accumulate(
            index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d>& pre, int idx){
                if (!effect_pts[idx]) return pre;
                total_res += errors[idx].dot(errors[idx]);
                effective_num++;
                return std::pair<Mat6d, Vec6d>(
                    pre.first + jacobians[idx].transpose() * jacobians[idx],
                    pre.second - jacobians[idx].transpose() * errors[idx]
                );
            });

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        Mat6d H = H_and_err.first;
        Vec6d err = H_and_err.second;

        HTVH.block<6,6>(0,0) = H * info_ratio;
        HTVr.segment<6>(0) = err * info_ratio;

        Vec6d dx = H.inverse() * err;

        // 更新副本 pose_iter
        pose_iter.so3() = pose_iter.so3() * SO3::exp(dx.head<3>());
        pose_iter.translation() += dx.tail<3>();

        LOG(INFO) << "iter " << iter << " total res: " << total_res
                  << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num
                  << ", dxn: " << dx.norm();

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose_iter).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }

    return true;
}

bool Icp3d::ComputeResidualAndJacobiansP2P2(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr) {
    LOG(INFO) << "aligning with point to point"; 
    assert(target_ != nullptr && source_ != nullptr);

    // 最终返回的 18-dim 矩阵 / 向量，初始化为 0
    HTVH.setZero();
    HTVr.setZero();
    const double info_ratio = 0.01;

    // 对源点云生成索引
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < (int)index.size(); ++i) index[i] = i;

    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
    std::vector<Vec3d> errors(index.size());

    // 复制一份 pose 用于迭代更新
    SE3 pose_iter = pose;

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {

        // 并行计算每个点的最近邻、残差与 3x6 雅可比
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose_iter * q; 
            std::vector<int> nn;
            kdtree_->GetClosestPoint(ToPointType(qs), nn, 1);

            if (!nn.empty()) {
                Vec3d p = ToVec3d(target_->points[nn[0]]);
                double dis2 = (p - qs).squaredNorm();
                if (dis2 > options_.max_nn_distance_) {
                    effect_pts[idx] = false;
                    return;
                }

                effect_pts[idx] = true;
                Vec3d e = p - qs;

                Eigen::Matrix<double, 3, 6> J;
                J.setZero();
                // 对旋转的偏导（dθ 在前 3），对平移的偏导（δt 在后 3）
                // 对 residual r = p - R*q - t 的线性化： δr ≈ R * hat(q) * dθ - δt
                J.block<3,3>(0,0) = pose_iter.so3().matrix() * SO3::hat(q); // ∂r/∂dθ
                J.block<3,3>(0,3) = -Mat3d::Identity();                     // ∂r/∂δt

                jacobians[idx] = J;
                errors[idx] = e;
            } else {
                effect_pts[idx] = false;
            }
        });

        // 累加得到 6x6 Hessian 与 6x1 b 
        double total_res = 0;
        int effective_num = 0;
        Mat6d H6 = Mat6d::Zero();
        Vec6d b6 = Vec6d::Zero();

        for (int idx : index) {
            if (!effect_pts[idx]) continue;
            total_res += errors[idx].dot(errors[idx]);
            effective_num++;
            H6 += jacobians[idx].transpose() * jacobians[idx];
            b6 += - jacobians[idx].transpose() * errors[idx];
        }

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        // --- 将 6x6, 6x1 嵌入到 18 维的 HTVH / HTVr 中（与 NDT 的状态布局一致）
        // 假定 18 维状态的索引：translation -> [0,1,2], rotation -> [6,7,8]
        // 我们的 6-vector dx6 = [dθ(3); δt(3)]，所以构造 18x6 的选择矩阵 B:
        // 列 0..2 (dθ) 映射到 18-vector 的 idx 6..8
        // 列 3..5 (δt) 映射到 18-vector 的 idx 0..2
        Eigen::Matrix<double, 18, 6> B;
        B.setZero();
        for (int i = 0; i < 3; ++i) {
            B(6 + i, i) = 1.0;   // dθ -> indices 6..8
            B(i, 3 + i) = 1.0;   // δt -> indices 0..2
        }

        // 每次迭代我们覆盖 HTVH/HTVr
        HTVH.setZero();
        HTVr.setZero();
        HTVH += B * H6 * B.transpose() * info_ratio;  // 18x18
        HTVr += B * b6 * info_ratio;                   // 18x1

        // 求解 6 维增量 (用 LDLT 更稳定)
        Vec6d dx6;
        Eigen::LDLT<Mat6d> ldlt(H6);
        if (ldlt.info() == Eigen::Success) {
            dx6 = ldlt.solve(b6);
        } else {
            // 退化情形的后备（可改为更好的正则化）
            dx6 = H6.inverse() * b6;
        }

        // 更新副本 pose_iter
        pose_iter.so3() = pose_iter.so3() * SO3::exp(dx6.head<3>());
        pose_iter.translation() += dx6.tail<3>();

        LOG(INFO) << "iter " << iter << " total res: " << total_res
                  << ", eff: " << effective_num
                  << ", mean res: " << (effective_num>0 ? total_res / effective_num : 0.0)
                  << ", dxn: " << dx6.norm();

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose_iter).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        if (dx6.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx6.transpose();
            break;
        }
    }

    return true;
}


bool Icp3d::ComputeResidualAndJacobiansP2Plane(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr) {
    LOG(INFO) << "aligning with point to plane";
    assert(target_ != nullptr && source_ != nullptr);
    // 整体流程与点到点 ICP 相同，主要区别在残差构造：点到平面残差
    HTVH.setZero();
    HTVr.setZero();
    const double info_ratio = 0.01;

    // if (!options_.use_initial_translation_) {
    //     pose.translation() = target_center_ - source_center_;  
    //     // 如果不用给的初始位移，就用点云质心差来初始化平移
    // }

    // 初始化索引数组 [0,1,2,...N-1]
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    // 每个源点对应的状态存储
    std::vector<bool> effect_pts(index.size(), false);              // 是否有效点
    std::vector<Eigen::Matrix<double, 1, 6>> jacobians(index.size()); // 雅可比 (1x6)
    std::vector<double> errors(index.size());    
    
    SE3 pose_iter = pose;

    // ----------------- ICP 主迭代循环 -----------------
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        // ---------- Step 1: 建立对应关系并计算残差 ----------
        // 最近邻搜索 + 平面拟合（可并行加速）
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);   // 源点
            Vec3d qs = pose_iter * q;                      // 转换到目标坐标系下

            std::vector<int> nn;
            kdtree_->GetClosestPoint(ToPointType(qs), nn, 5);  // 找 5 个最近邻目标点
            if (nn.size() > 3) {
                // 转为 Eigen 向量
                std::vector<Vec3d> nn_eigen;
                for (int i = 0; i < nn.size(); ++i) {
                    nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
                }

                Vec4d n;
                if (!FitPlane(nn_eigen, n)) {   
                    // 用邻域点拟合平面，n=(nx,ny,nz,d)，表示 nx*x+ny*y+nz*z+d=0
                    effect_pts[idx] = false;
                    return;
                }

                double dis = n.head<3>().dot(qs) + n[3];  //// 点 qs 到拟合平面的距离，n[3]就是d
                

                if (fabs(dis) > options_.max_plane_distance_) {
                    // 距离过大，认为是错误匹配，丢弃
                    effect_pts[idx] = false;
                    return;
                }

                // 有效点，保存残差与雅可比
                effect_pts[idx] = true;

                // --------- 残差构造 ---------
                // 点到平面残差: ei = n^T (Rp + t - q_plane)
                // 对位姿参数 ξ 的导数（李代数扰动模型）
                Eigen::Matrix<double, 1, 6> J;
                J.block<1, 3>(0, 0) = -n.head<3>().transpose() * pose_iter.so3().matrix() * SO3::hat(q); 
                // 对旋转的导数
                J.block<1, 3>(0, 3) = n.head<3>().transpose();  
                // 对平移的导数

                jacobians[idx] = J;   // 存储雅可比
                errors[idx] = dis;    // 存储残差
            } else {
                effect_pts[idx] = false;
            }
        });

        // ---------- Step 2: 构建正规方程 ----------
        // 累加 Hessian H=Σ(JᵢᵀJᵢ) 和右端项 b=Σ(-Jᵢᵀ eᵢ)
        double total_res = 0;    // 残差平方和
        int effective_num = 0;   // 有效点数
        auto H_and_err = std::accumulate(
            index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d>& pre,
                                                                           int idx) -> std::pair<Mat6d, Vec6d> {
                if (!effect_pts[idx]) {
                    return pre;  // 无效点，跳过
                } else {
                    total_res += errors[idx] * errors[idx];  // 累加误差平方
                    effective_num++;
                    return std::pair<Mat6d, Vec6d>(
                        pre.first + jacobians[idx].transpose() * jacobians[idx],     // H += JᵀJ
                        pre.second - jacobians[idx].transpose() * errors[idx]        // b += -Jᵀe
                    );
                }
            });

        // 检查是否有足够的约束点
        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        // ---------- Step 3: 解正规方程 ----------
        Mat6d H = H_and_err.first;   // Hessian 近似，曲率矩阵
        Vec6d err = H_and_err.second; // -Jᵀe，相当于负梯度方向
        Vec6d dx = H.inverse() * err; // 求解增量 δξ = H⁻¹(-Jᵀe)

        HTVH.block<6,6>(0,0) = H * info_ratio;
        HTVr.segment<6>(0) = err * info_ratio;

        // ---------- Step 4: 更新位姿 ----------
        pose_iter.so3() = pose_iter.so3() * SO3::exp(dx.head<3>());  // 更新旋转
        pose_iter.translation() += dx.tail<3>();                // 更新平移

        // ---------- Step 5: 日志与收敛检查 ----------
        LOG(INFO) << "iter " << iter 
                  << " total res: " << total_res 
                  << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num 
                  << ", dxn: " << dx.norm();

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        if (dx.norm() < options_.eps_) {  // 增量足够小，认为收敛
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }

    // 最终结果回写
    return true;
}

bool Icp3d::ComputeResidualAndJacobiansP2Plane2(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr) {
    LOG(INFO) << "aligning with point to plane";
    assert(target_ != nullptr && source_ != nullptr);

    // 初始化 18-d 输出
    HTVH.setZero();
    HTVr.setZero();
    const double info_ratio = 0.01;

    // 索引数组
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < (int)index.size(); ++i) index[i] = i;

    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 1, 6>> jacobians(index.size());
    std::vector<double> errors(index.size());

    SE3 pose_iter = pose;

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        // 并行：最近邻搜索 -> 平面拟合 -> 计算残差与 1x6 雅可比
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose_iter * q;

            std::vector<int> nn;
            kdtree_->GetClosestPoint(ToPointType(qs), nn, 5); // 取 5 个邻居进行拟合
            if (nn.size() > 3) {
                // 转为 Eigen
                std::vector<Vec3d> nn_eigen;
                nn_eigen.reserve(nn.size());
                for (int ii = 0; ii < (int)nn.size(); ++ii) {
                    nn_eigen.emplace_back(ToVec3d(target_->points[nn[ii]]));
                }

                Vec4d plane;
                if (!FitPlane(nn_eigen, plane)) {
                    effect_pts[idx] = false;
                    return;
                }

                // plane: [nx, ny, nz, d], 假设法向已归一化（请确保 FitPlane 的约定）
                Vec3d n = plane.head<3>();
                double d = plane[3];

                double dis = n.dot(qs) + d; // 点到平面有符号距离

                if (fabs(dis) > options_.max_plane_distance_) {
                    effect_pts[idx] = false;
                    return;
                }

                // 构造雅可比（1x6），状态顺序为 [dθ(3), δt(3)]
                // 注意符号与你原始写法保持一致：
                // ∂(n^T (R q + t) + d)/∂δθ ≈ - n^T * R * hat(q)
                // ∂(...)/∂δt = n^T
                Eigen::Matrix<double, 1, 6> J;
                J.setZero();
                J.block<1,3>(0,0) = - n.transpose() * pose_iter.so3().matrix() * SO3::hat(q); // 1x3
                J.block<1,3>(0,3) = n.transpose();                                           // 1x3

                effect_pts[idx] = true;
                jacobians[idx] = J;
                errors[idx] = dis;
            } else {
                effect_pts[idx] = false;
            }
        });

        // 串行累加 6x6 Hessian 与 6x1 b（b = Σ -Jᵀ e）
        double total_res = 0.0;
        int effective_num = 0;
        Mat6d H6 = Mat6d::Zero();
        Vec6d b6 = Vec6d::Zero();

        for (int idx : index) {
            if (!effect_pts[idx]) continue;
            double e = errors[idx];
            total_res += e * e;
            effective_num++;

            // jacobians[idx] is 1x6
            H6 += jacobians[idx].transpose() * jacobians[idx];  // 6x6
            b6 += - jacobians[idx].transpose() * e;             // 6x1
        }

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        // 嵌入到 18 维布局，与 NDT 保持一致（旋转 6..8，平移 0..2）
        Eigen::Matrix<double, 18, 6> B;
        B.setZero();
        for (int i = 0; i < 3; ++i) {
            B(6 + i, i) = 1.0;  // dθ -> 6..8
            B(i, 3 + i) = 1.0;  // δt -> 0..2
        }

        HTVH.setZero();
        HTVr.setZero();
        HTVH += B * H6 * B.transpose() * info_ratio;
        HTVr += B * b6 * info_ratio;

        // 求解 6 维增量（优先用 LDLT，失败时带小阻尼回退）
        Vec6d dx6;
        Eigen::LDLT<Mat6d> ldlt(H6);
        if (ldlt.info() == Eigen::Success) {
            dx6 = ldlt.solve(b6);
        } else {
            // 退化处理：对角加小阻尼（LM 风格）
            const double lambda = 1e-6;
            Mat6d H6_damped = H6;
            H6_damped.diagonal().array() += lambda;
            dx6 = H6_damped.ldlt().solve(b6);
        }

        // 更新 pose_iter（保持和点到点实现一致的状态更新顺序）
        pose_iter.so3() = pose_iter.so3() * SO3::exp(dx6.head<3>());
        pose_iter.translation() += dx6.tail<3>();

        LOG(INFO) << "iter " << iter
                  << " total res: " << total_res
                  << ", eff: " << effective_num
                  << ", mean res: " << (effective_num>0 ? total_res / effective_num : 0.0)
                  << ", dxn: " << dx6.norm();

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose_iter).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        if (dx6.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx6.transpose();
            break;
        }
    }

    return true;
}

bool Icp3d::ComputeResidualAndJacobiansP2L(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr){
    LOG(INFO) << "aligning with point to line";  // 输出信息：开始点到直线的配准
    assert(target_ != nullptr && source_ != nullptr);  // 确保目标点云和源点云不为空

    HTVH.setZero();
    HTVr.setZero();
    const double info_ratio = 0.01;
    SE3 pose_iter = pose;

    // 初始化位姿
    
    // if (options_.use_initial_translation_) {
    //     // 1. 使用初始平移，将源点云中心对齐到目标点云中心
    //     pose.translation() = target_center_ - source_center_;
    //     LOG(INFO) << "init trans set to " << pose.translation().transpose();
    // }

    // 构建源点云点索引
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    // 标记有效点、存储雅可比矩阵和误差
    std::vector<bool> effect_pts(index.size(), false);  // 标记当前点是否参与迭代
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());  // 每个点的雅可比
    std::vector<Vec3d> errors(index.size());  // 每个点的误差

    // 迭代求解
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        // gauss-newton 迭代
        // 最近邻搜索，可并行
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);  // 获取源点
            Vec3d qs = pose_iter * q;                     // 转换到当前位姿下
            std::vector<int> nn;
            kdtree_->GetClosestPoint(ToPointType(qs), nn, 5);  // 查找5个最近邻

            if (nn.size() == 5) {
                // 最近邻转换为Eigen格式
                std::vector<Vec3d> nn_eigen;
                for (int i = 0; i < 5; ++i) {
                    nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
                }

                // 拟合直线
                Vec3d d, p0;
                if (!FitLine(nn_eigen, p0, d, options_.max_line_distance_)) {
                    // 拟合失败，该点不参与
                    effect_pts[idx] = false;
                    return;
                }

                // 计算点到直线的误差，方向向量d的反对称矩阵叉乘向量v
                Vec3d err = SO3::hat(d) * (qs - p0); 

                if (err.norm() > options_.max_line_distance_) {
                    // 点离直线太远，不参与
                    effect_pts[idx] = false;
                    return;
                }

                effect_pts[idx] = true;  // 标记为有效点

                // 构建误差的雅可比矩阵
                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = -SO3::hat(d) * pose_iter.so3().matrix() * SO3::hat(q);  // 旋转部分
                J.block<3, 3>(0, 3) = SO3::hat(d);                                        // 平移部分

                jacobians[idx] = J;  // 保存雅可比
                errors[idx] = err;   // 保存误差
            } else {
                effect_pts[idx] = false;  // 最近邻不足5个，标记为无效
            }
        });

        // 累加Hessian矩阵和误差向量
        double total_res = 0;  // 总残差平方和
        int effective_num = 0; // 有效点数量
        auto H_and_err = std::accumulate(
            index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d>& pre,
                                                                           int idx) -> std::pair<Mat6d, Vec6d> {
                if (!effect_pts[idx]) {
                    return pre;  // 无效点不参与累加
                } else {
                    total_res += errors[idx].dot(errors[idx]);  // 累加残差平方
                    effective_num++;
                    return std::pair<Mat6d, Vec6d>(
                        pre.first + jacobians[idx].transpose() * jacobians[idx],  // Hessian累加 H
                        pre.second - jacobians[idx].transpose() * errors[idx]     // 误差累加 b
                    );
                }
            });

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;  // 有效点过少，无法迭代
            return false;
        }

        Mat6d H = H_and_err.first;  // Hessian
        Vec6d err = H_and_err.second; // 误差向量

        HTVH.block<6,6>(0,0) = H * info_ratio;
        HTVr.segment<6>(0) = err * info_ratio;

        // 求解位姿增量 dx
        Vec6d dx = H.inverse() * err;
        pose_iter.so3() = pose_iter.so3() * SO3::exp(dx.head<3>()); // 更新旋转
        pose_iter.translation() += dx.tail<3>();               // 更新平移

        // 如果有地面真实位姿，输出误差
        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose_iter).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        // 输出当前迭代信息
        LOG(INFO) << "iter " << iter << " total res: " << total_res
                  << ", eff: " << effective_num
                  << ", mean res: " << total_res / effective_num
                  << ", dxn: " << dx.norm();

        // 判断是否收敛
        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }

    return true;
}

bool Icp3d::ComputeResidualAndJacobiansP2L2(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr){
    LOG(INFO) << "aligning with point to line";
    assert(target_ != nullptr && source_ != nullptr);

    HTVH.setZero();
    HTVr.setZero();
    const double info_ratio = 0.01;
    SE3 pose_iter = pose;

    // 索引
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < (int)index.size(); ++i) index[i] = i;

    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
    std::vector<Vec3d> errors(index.size());

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        // 并行：最近邻搜索 + 直线拟合 + 计算 3x6 雅可比与 3x1 残差
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose_iter * q;
            std::vector<int> nn;
            kdtree_->GetClosestPoint(ToPointType(qs), nn, 5); // 取 5 个邻居

            if (nn.size() == 5) {
                std::vector<Vec3d> nn_eigen;
                nn_eigen.reserve(5);
                for (int i = 0; i < 5; ++i) nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));

                Vec3d d, p0;
                if (!FitLine(nn_eigen, p0, d, options_.max_line_distance_)) {
                    effect_pts[idx] = false;
                    return;
                }

                // 误差为点到直线的向量（沿垂直方向）
                Vec3d err = SO3::hat(d) * (qs - p0);

                if (err.norm() > options_.max_line_distance_) {
                    effect_pts[idx] = false;
                    return;
                }

                // 雅可比：对状态 xi = [dθ, δt]，误差 r = hat(d)*(R q + t - p0)
                Eigen::Matrix<double, 3, 6> J;
                J.setZero();
                // ∂r/∂dθ ≈ - hat(d) * R * hat(q)
                J.block<3,3>(0,0) = - SO3::hat(d) * pose_iter.so3().matrix() * SO3::hat(q);
                // ∂r/∂δt = hat(d)
                J.block<3,3>(0,3) = SO3::hat(d);

                effect_pts[idx] = true;
                jacobians[idx] = J;
                errors[idx] = err;
            } else {
                effect_pts[idx] = false;
            }
        });

        // 串行累加 6x6 Hessian 与 6x1 b (b = Σ -Jᵀ e)
        double total_res = 0.0;
        int effective_num = 0;
        Mat6d H6 = Mat6d::Zero();
        Vec6d b6 = Vec6d::Zero();

        for (int idx : index) {
            if (!effect_pts[idx]) continue;
            total_res += errors[idx].dot(errors[idx]);
            effective_num++;
            H6 += jacobians[idx].transpose() * jacobians[idx];
            b6 += - jacobians[idx].transpose() * errors[idx];
        }

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        // 将 6x6 嵌入到 18x18（与 NDT 状态布局一致：rotation -> 6..8, translation -> 0..2）
        Eigen::Matrix<double, 18, 6> B;
        B.setZero();
        for (int i = 0; i < 3; ++i) {
            B(6 + i, i) = 1.0;   // dθ -> 6..8
            B(i, 3 + i) = 1.0;   // δt -> 0..2
        }

        HTVH.setZero();
        HTVr.setZero();
        HTVH += B * H6 * B.transpose() * info_ratio;
        HTVr += B * b6 * info_ratio;

        // 求解 6 维增量 dx6（优先使用 LDLT，失败时带阻尼回退）
        Vec6d dx6;
        Eigen::LDLT<Mat6d> ldlt(H6);
        if (ldlt.info() == Eigen::Success) {
            dx6 = ldlt.solve(b6);
        } else {
            const double lambda = 1e-6;
            Mat6d H6_damped = H6;
            H6_damped.diagonal().array() += lambda;
            dx6 = H6_damped.ldlt().solve(b6);
        }

        // 更新 pose_iter（旋转部分用左乘增量）
        pose_iter.so3() = pose_iter.so3() * SO3::exp(dx6.head<3>());
        pose_iter.translation() += dx6.tail<3>();

        LOG(INFO) << "iter " << iter << " total res: " << total_res
                  << ", eff: " << effective_num
                  << ", mean res: " << (effective_num>0 ? total_res / effective_num : 0.0)
                  << ", dxn: " << dx6.norm();

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose_iter).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        if (dx6.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx6.transpose();
            break;
        }
    }

    return true;
}




} // namespace sad

