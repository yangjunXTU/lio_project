/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-10-22 08:10:46
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-10-29 09:25:58
 * @FilePath: /lio_project/src/vio.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "utils/vio.h"

#include <algorithm>
#include <limits>

namespace {

struct GridCandidate {
    PTs_3d2 point;
    double score = 0.0;
};

inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;
    if (y >= img.rows) y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - xx) * yy * data[img.step] +
        xx * yy * data[img.step + 1]
    );
}

inline float GetImageGradientMagnitude(const cv::Mat &img, float x, float y) {
    const float dx = 0.5f * (GetPixelValue(img, x + 1.0f, y) - GetPixelValue(img, x - 1.0f, y));
    const float dy = 0.5f * (GetPixelValue(img, x, y + 1.0f) - GetPixelValue(img, x, y - 1.0f));
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

VIO::VIO(/* args */)
{
    buildPatchPattern();
}


VIO::~VIO()
{
}


void VIO::processFrame(cv::Mat &img, SE3 &T_w_i_meas)
{
    visual_measurement_ready_ = false;
    resetTrackingStats();
    
    if (img.empty()) printf("[ VIO ] Empty Image!\n");
    // img_rgb = img.clone();
    // img_cp = img.clone();
    
    if (img.channels() == 3) cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    
    new_frame_.reset(new Frame(img));
    new_frame_->BuildPyramid(pyr_levels_);
    // new_frame_->T_c_w = nav_state_w_i_ptr_->GetSE3();
    ROS_INFO("--------------------------------------------7");
    setCameraPoseFromNavState(*nav_state_w_i_ptr_);
    ROS_INFO("--------------------------------------------8");
    buildVisibleMapPoints(*new_frame_);
    ROS_INFO("--------------------------------------------9");
    if(flg_first_scan_vio)
    {
      last_frame_ = new_frame_;
      // last_frame_.reset(new_frame_.release());
      flg_first_scan_vio = false;
      visual_measurement_ready_ = false;
      return;
    }
    if (!last_frame_ || last_frame_->pts.empty() || new_frame_->pts.empty()) {
      ROS_WARN("[VIO] skipped frame because reference/current visual points are insufficient. ref=%zu cur=%zu",
               last_frame_ ? last_frame_->pts.size() : 0, new_frame_->pts.size());
      visual_measurement_ready_ = false;
      return;
    }
    stateEstimate();
    ROS_INFO("--------------------------------------------10");
    if (!new_frame_) {
        ROS_ERROR("new_frame_ is null!");
        return;
    }
    T_w_i_meas = new_frame_->T_w_i_meas;
    visual_measurement_ready_ = true;
    ROS_INFO("get {%.d} frame, this frame id = {%d} ;", new_frame_->frame_counter_,new_frame_->id_);
}

void VIO::setImuToLidarExtrinsic(const Vec3d &transl, const Mat3d &rot)
{
  t_l_i = -rot.transpose() * transl;
  R_l_i = rot.transpose();
}

void VIO::setLidarToCameraExtrinsic(const Mat3d &R_c_l_in, const Vec3d &t_c_l_in)
{
  R_c_l = R_c_l_in;
  t_c_l = t_c_l_in;
}

void VIO::initializeVIO()
{
    R_c_i = R_c_l * R_l_i;
    t_c_i = R_c_l * t_l_i + t_c_l;
    T_c_i = SE3(R_c_i, t_c_i);
    publishStaticTransformC2I();
}

void VIO::setCameraParameter(const CameraConfig& camera_config)
{
    cam_model_ = camera_config.model;
    cam_fx_ = camera_config.K(0, 0);
    cam_fy_ = camera_config.K(1, 1);
    cam_cx_ = camera_config.K(0, 2);
    cam_cy_ = camera_config.K(1, 2);
    cam_width_ = camera_config.width;
    cam_height_ = camera_config.height;
    scale_ = camera_config.scale;

    cam_d0_ = camera_config.D.size() > 0 ? camera_config.D[0] : 0.0;
    cam_d1_ = camera_config.D.size() > 1 ? camera_config.D[1] : 0.0;
    cam_d2_ = camera_config.D.size() > 2 ? camera_config.D[2] : 0.0;
    cam_d3_ = camera_config.D.size() > 3 ? camera_config.D[3] : 0.0;
    cam_d4_ = camera_config.D.size() > 4 ? camera_config.D[4] : 0.0;
 
}

void VIO::setVioParameter(const VioConfig& vio_config)
{
    pyr_levels_ = std::max(1, vio_config.pyr_levels);
    half_patch_size_ = std::max(1, vio_config.half_patch_size);
    max_visual_points_ = std::max(20, vio_config.max_points);
    gn_iters_per_level_ = std::max(1, vio_config.gn_iters_per_level);
    min_valid_residuals_ = std::max(1, vio_config.min_valid_residuals);
    grid_rows_ = std::max(1, vio_config.grid_rows);
    grid_cols_ = std::max(1, vio_config.grid_cols);
    max_points_per_cell_ = std::max(1, vio_config.max_points_per_cell);
    min_gradient_ = std::max(0.0, vio_config.min_gradient);
    min_depth_ = std::max(1e-3, vio_config.min_depth);
    max_depth_ = std::max(min_depth_ + 1e-3, vio_config.max_depth);
    huber_delta_ = std::max(1e-6, vio_config.huber_delta);
    info_scale_ = std::max(1e-8, vio_config.info_scale);
    min_inlier_ratio_ = std::max(0.0, std::min(1.0, vio_config.min_inlier_ratio));
    max_mean_residual_ = std::max(1e-6, vio_config.max_mean_residual);
    max_update_translation_ = std::max(1e-6, vio_config.max_update_translation);
    max_update_rotation_deg_ = std::max(1e-6, vio_config.max_update_rotation_deg);
    min_keyframe_translation_ = std::max(0.0, vio_config.min_keyframe_translation);
    min_keyframe_rotation_deg_ = std::max(0.0, vio_config.min_keyframe_rotation_deg);
    buildPatchPattern();
}

void VIO::buildPatchPattern()
{
  patch_pattern_.clear();
  for (int dx = -half_patch_size_; dx <= half_patch_size_; ++dx) {
    for (int dy = -half_patch_size_; dy <= half_patch_size_; ++dy) {
      patch_pattern_.emplace_back(dx, dy);
    }
  }
}

void VIO::setCameraPoseFromNavState(NavStated nav_state_w_i)
{
  Mat3d R_w_i = nav_state_w_i.R_.matrix();
  Vec3d t_w_i = nav_state_w_i.p_.matrix();

//   M3D R_w_i(state.rot_end);
//   V3D t_w_i(state.pos_end);
  R_c_w = R_c_i * R_w_i.transpose();
  t_c_w = -R_c_i * R_w_i.transpose() * t_w_i + t_c_i;
  new_frame_->T_c_w = SE3(R_c_w, t_c_w);
  
}

void VIO::resetTrackingStats()
{
  tracking_stats_ = VisualTrackingStats();
}

bool VIO::ShouldAcceptVisualUpdate() const
{
  if (tracking_stats_.selected_points <= 0) {
    return false;
  }
  if (tracking_stats_.valid_residuals < min_valid_residuals_) {
    return false;
  }
  if (tracking_stats_.inlier_ratio < min_inlier_ratio_) {
    return false;
  }
  if (tracking_stats_.mean_abs_residual > max_mean_residual_) {
    return false;
  }
  if (tracking_stats_.update_translation_norm > max_update_translation_) {
    return false;
  }
  if (tracking_stats_.update_rotation_deg > max_update_rotation_deg_) {
    return false;
  }
  return true;
}

bool VIO::ShouldPromoteReferenceFrame() const
{
  if (!tracking_stats_.update_accepted) {
    return false;
  }
  return tracking_stats_.reference_translation_norm >= min_keyframe_translation_ ||
         tracking_stats_.reference_rotation_deg >= min_keyframe_rotation_deg_;
}

double VIO::ComputeReferenceMotionTranslation() const
{
  if (!last_frame_ || !new_frame_) {
    return 0.0;
  }
  SE3 T_rel = new_frame_->T_c_w * last_frame_->T_c_w.inverse();
  return T_rel.translation().norm();
}

double VIO::ComputeReferenceMotionRotationDeg() const
{
  if (!last_frame_ || !new_frame_) {
    return 0.0;
  }
  SE3 T_rel = new_frame_->T_c_w * last_frame_->T_c_w.inverse();
  return T_rel.so3().log().norm() * 180.0 / M_PI;
}

void VIO::finalizeFrame() {
  if (new_frame_ && tracking_stats_.promote_reference) {
    last_frame_ = new_frame_;
  }
  visual_measurement_ready_ = false;
}


void VIO::publishStaticTransformC2I() {

  geometry_msgs::TransformStamped static_transformStamped;
  
  // 设置时间戳
  static_transformStamped.header.stamp = ros::Time::now();
  
  // 设置父坐标系（IMU坐标系）
  static_transformStamped.header.frame_id = "aft_mapped";
  
  // 设置子坐标系（相机坐标系）
  static_transformStamped.child_frame_id = "camera_vio";
  
  auto R_i_c = R_c_i.transpose();
  auto t_i_c = -R_c_i.transpose() * t_c_i;
  // 设置平移
  static_transformStamped.transform.translation.x = t_i_c.x();
  static_transformStamped.transform.translation.y = t_i_c.y();
  static_transformStamped.transform.translation.z = t_i_c.z();
  
  // 将旋转矩阵转换为四元数
  Eigen::Quaterniond quat(R_i_c);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  
  // 发布静态变换
  static_broadcaster_.sendTransform(static_transformStamped);
  
  ROS_INFO("Published static TF from %s to %s", 
            static_transformStamped.header.frame_id.c_str(),
            static_transformStamped.child_frame_id.c_str());
}

void VIO::buildVisibleMapPoints(Frame &new_frame)
{
  new_frame.pts.clear();
  tracking_stats_.candidate_points = 0;
  tracking_stats_.selected_points = 0;
  const double scale_factor = 1.0 / static_cast<double>(1 << (pyr_levels_ - 1));
  const int margin = half_patch_size_ * (1 << (pyr_levels_ - 1)) + 2;
  std::vector<std::vector<GridCandidate>> grid_buckets(static_cast<size_t>(grid_rows_ * grid_cols_));

  for (const auto& point : *vio_scan_world)
  {
    Vec3d pt_w;
    pt_w << point.x, point.y, point.z;

    Vec3d pt_c(new_frame.worldToCamera(pt_w));
    if (pt_c.z() <= 0 || pt_c.z() < min_depth_ || pt_c.z() > max_depth_) {
      continue;
    }
    Vec2d px = cameraPointToPixel(pt_c);
    if(!isUVEffective(px))
    {
      continue;
    }
    if (!IsPointPhotometricallyValid(new_frame, pt_c, px, margin)) {
      continue;
    }

    const double gradient = GetImageGradientMagnitude(new_frame.img_, px[0], px[1]);
    if (gradient < min_gradient_) {
      continue;
    }

    tracking_stats_.candidate_points++;

    PTs_3d2 pt;
    pt.pt_w = pt_w;
    pt.px_ref = px;

    const int grid_x = std::min(grid_cols_ - 1, std::max(0, static_cast<int>(px[0] * grid_cols_ / new_frame.img_.cols)));
    const int grid_y = std::min(grid_rows_ - 1, std::max(0, static_cast<int>(px[1] * grid_rows_ / new_frame.img_.rows)));
    const size_t grid_index = static_cast<size_t>(grid_y * grid_cols_ + grid_x);
    grid_buckets[grid_index].push_back(GridCandidate{pt, ScoreVisualPoint(pt_c, gradient)});
  }

  std::vector<GridCandidate> selected_candidates;
  selected_candidates.reserve(max_visual_points_);
  for (auto& bucket : grid_buckets) {
    if (bucket.empty()) {
      continue;
    }
    std::sort(bucket.begin(), bucket.end(), [](const GridCandidate& lhs, const GridCandidate& rhs) {
      return lhs.score > rhs.score;
    });
    const int keep_num = std::min<int>(max_points_per_cell_, bucket.size());
    for (int i = 0; i < keep_num; ++i) {
      selected_candidates.push_back(bucket[i]);
    }
  }

  if (selected_candidates.size() > static_cast<size_t>(max_visual_points_)) {
    std::sort(selected_candidates.begin(), selected_candidates.end(),
              [](const GridCandidate& lhs, const GridCandidate& rhs) { return lhs.score > rhs.score; });
    selected_candidates.resize(max_visual_points_);
  }

  new_frame.pts.reserve(selected_candidates.size());
  for (const auto& candidate : selected_candidates) {
    new_frame.pts.push_back(candidate.point);
  }
  tracking_stats_.selected_points = static_cast<int>(new_frame.pts.size());
  ROS_INFO("new_frame.pts.size = %ld",new_frame.pts.size());

  // drawimg();
  
  return;

}

double VIO::ScoreVisualPoint(const Vec3d& pt_c, double gradient) const
{
  const double depth_score = 1.0 / std::max(pt_c.z(), 1e-3);
  return gradient + 5.0 * depth_score;
}

bool VIO::IsPointPhotometricallyValid(const Frame& frame, const Vec3d& pt_c, const Vec2d& px, int margin) const
{
  if (pt_c.z() <= 0) {
    return false;
  }
  if (px[0] < margin || px[0] >= frame.img_.cols - margin ||
      px[1] < margin || px[1] >= frame.img_.rows - margin) {
    return false;
  }

  const double scale_factor = 1.0 / static_cast<double>(1 << (pyr_levels_ - 1));
  const Vec2d px_coarse = px * scale_factor;
  if (px_coarse[0] < half_patch_size_ + 1 || px_coarse[0] >= frame.pyr_.back().cols - half_patch_size_ - 1 ||
      px_coarse[1] < half_patch_size_ + 1 || px_coarse[1] >= frame.pyr_.back().rows - half_patch_size_ - 1) {
    return false;
  }

  return true;
}

void VIO::stateEstimate()
{
  if(last_frame_)
  {
    DirectPoseEstimationMultiLayer();
    new_frame_->T_w_i_meas = new_frame_->T_c_w.inverse() * T_c_i; // * new_frame_->T_c_w;
  }
  return;
  
}

void VIO::ComputeResidualAndJacobians(const SE3 &T_w_i, Mat18d &HTVH, Vec18d &HTVr)
{
  HTVH.setZero();
  HTVr.setZero();
  tracking_stats_.total_residuals = 0;
  tracking_stats_.valid_residuals = 0;
  tracking_stats_.inlier_residuals = 0;
  tracking_stats_.mean_cost = 0.0;
  tracking_stats_.mean_abs_residual = 0.0;
  tracking_stats_.inlier_ratio = 0.0;
  tracking_stats_.update_translation_norm = 0.0;
  tracking_stats_.update_rotation_deg = 0.0;
  tracking_stats_.reference_translation_norm = ComputeReferenceMotionTranslation();
  tracking_stats_.reference_rotation_deg = ComputeReferenceMotionRotationDeg();
  tracking_stats_.update_accepted = false;
  tracking_stats_.promote_reference = false;

  if (!last_frame_ || !new_frame_) {
    return;
  }
  if (last_frame_->pts.empty()) {
    return;
  }

  Mat6d H_pose = Mat6d::Zero();
  Vec6d b_pose = Vec6d::Zero();
  int valid_residuals = 0;
  int inlier_residuals = 0;
  int total_residuals = 0;
  double total_cost = 0.0;
  double total_abs_residual = 0.0;

  for (int level = pyr_levels_ - 1; level >= 0; --level) {
    const double scale_factor = 1.0 / static_cast<double>(1 << level);
    const cv::Mat& ref_img = last_frame_->pyr_[level];
    const cv::Mat& cur_img = new_frame_->pyr_[level];
    const double level_weight = 1.0 / static_cast<double>(1 << level);
    SE3 T_c_w_eval = T_c_i * T_w_i.inverse();

    for (const auto& pt : last_frame_->pts) {
      Vec3d pt_c = T_c_w_eval * pt.pt_w;
      Vec2d px_cur = cameraPointToPixel(pt_c, scale_factor);
      Vec2d px_ref = pt.px_ref * scale_factor;
      if (!isUVEffective(px_cur)) {
        continue;
      }

      if (px_cur[0] < half_patch_size_ || px_cur[0] >= cur_img.cols - half_patch_size_ ||
          px_cur[1] < half_patch_size_ || px_cur[1] >= cur_img.rows - half_patch_size_ ||
          px_ref[0] < half_patch_size_ || px_ref[0] >= ref_img.cols - half_patch_size_ ||
          px_ref[1] < half_patch_size_ || px_ref[1] >= ref_img.rows - half_patch_size_) {
        continue;
      }

      for (const auto& offset : patch_pattern_) {
          total_residuals++;
          const int dx = offset[0];
          const int dy = offset[1];
          const double ref_intensity = GetPixelValue(ref_img, px_ref[0] + dx, px_ref[1] + dy);
          const double cur_intensity = GetPixelValue(cur_img, px_cur[0] + dx, px_cur[1] + dy);
          const double residual = ref_intensity - cur_intensity;
          const Eigen::Vector2d J_img_pixel = ComputeImageGradient(cur_img, px_cur[0] + dx, px_cur[1] + dy);
          const Eigen::Matrix<double, 2, 3> J_proj = ComputeProjectionJacobian(pt_c, scale_factor);
          const Eigen::Matrix<double, 3, 6> J_pose = ComputePointJacobianWrtPose(T_w_i, pt.pt_w);
          const Eigen::Matrix<double, 1, 6> J6 = -J_img_pixel.transpose() * J_proj * J_pose;
          Eigen::Matrix<double, 1, 18> J18 = Eigen::Matrix<double, 1, 18>::Zero();
          J18.block<1, 3>(0, 0) = J6.block<1, 3>(0, 0);
          J18.block<1, 3>(0, 6) = J6.block<1, 3>(0, 3);

          double weight = 1.0;
          const double abs_residual = std::abs(residual);
          if (abs_residual > huber_delta_) {
            weight = huber_delta_ / abs_residual;
          } else {
            inlier_residuals++;
          }

          HTVH += J18.transpose() * J18 * weight * info_scale_ * level_weight;
          HTVr += -J18.transpose() * residual * weight * info_scale_ * level_weight;
          H_pose += J6.transpose() * J6 * weight * level_weight;
          b_pose += -J6.transpose() * residual * weight * level_weight;
          total_cost += residual * residual;
          total_abs_residual += abs_residual;
          valid_residuals++;
      }
    }
  }

  tracking_stats_.total_residuals = total_residuals;
  tracking_stats_.valid_residuals = valid_residuals;
  tracking_stats_.inlier_residuals = inlier_residuals;
  if (valid_residuals > 0) {
    tracking_stats_.mean_cost = total_cost / valid_residuals;
    tracking_stats_.mean_abs_residual = total_abs_residual / valid_residuals;
    tracking_stats_.inlier_ratio = static_cast<double>(inlier_residuals) / valid_residuals;
  }

  if (valid_residuals >= min_valid_residuals_ && H_pose.diagonal().minCoeff() > 1e-9) {
    Vec6d pose_update = H_pose.ldlt().solve(b_pose);
    if (pose_update.allFinite()) {
      tracking_stats_.update_translation_norm = pose_update.head<3>().norm();
      tracking_stats_.update_rotation_deg = pose_update.tail<3>().norm() * 180.0 / M_PI;
    }
  }

  tracking_stats_.update_accepted = ShouldAcceptVisualUpdate();
  tracking_stats_.promote_reference = ShouldPromoteReferenceFrame();

  if (tracking_stats_.update_accepted) {
    ROS_INFO("[VIO] accepted: valid=%d inlier_ratio=%.3f mean_abs=%.3f dtrans=%.4f drot=%.3f deg promote=%d",
             tracking_stats_.valid_residuals, tracking_stats_.inlier_ratio, tracking_stats_.mean_abs_residual,
             tracking_stats_.update_translation_norm, tracking_stats_.update_rotation_deg,
             tracking_stats_.promote_reference);
  } else {
    ROS_WARN("[VIO] rejected: selected=%d valid=%d inlier_ratio=%.3f mean_abs=%.3f dtrans=%.4f drot=%.3f deg",
             tracking_stats_.selected_points, tracking_stats_.valid_residuals, tracking_stats_.inlier_ratio,
             tracking_stats_.mean_abs_residual, tracking_stats_.update_translation_norm,
             tracking_stats_.update_rotation_deg);
    HTVH.setZero();
    HTVr.setZero();
  }
}

Eigen::Vector2d VIO::ComputeImageGradient(const cv::Mat& img, double u, double v) const
{
  return Eigen::Vector2d(
      0.5 * (GetPixelValue(img, u + 1.0, v) - GetPixelValue(img, u - 1.0, v)),
      0.5 * (GetPixelValue(img, u, v + 1.0) - GetPixelValue(img, u, v - 1.0)));
}

Eigen::Matrix<double, 2, 3> VIO::ComputeProjectionJacobian(const Vec3d& pt_c, double scale_factor) const
{
  Eigen::Matrix<double, 2, 3> J_proj = Eigen::Matrix<double, 2, 3>::Zero();
  const double X = pt_c.x();
  const double Y = pt_c.y();
  const double Z = pt_c.z();
  const double inv_Z = 1.0 / Z;
  const double inv_Z2 = inv_Z * inv_Z;
  const double fx = cam_fx_ * scale_factor;
  const double fy = cam_fy_ * scale_factor;
  J_proj(0, 0) = fx * inv_Z;
  J_proj(0, 2) = -fx * X * inv_Z2;
  J_proj(1, 1) = fy * inv_Z;
  J_proj(1, 2) = -fy * Y * inv_Z2;
  return J_proj;
}

Eigen::Matrix<double, 3, 6> VIO::ComputePointJacobianWrtPose(const SE3& T_w_i, const Vec3d& pt_w) const
{
  Eigen::Matrix<double, 3, 6> J_pose = Eigen::Matrix<double, 3, 6>::Zero();
  const Mat3d R_w_i = T_w_i.so3().matrix();
  const Vec3d t_w_i = T_w_i.translation();
  const Vec3d pt_i = R_w_i.transpose() * (pt_w - t_w_i);
  J_pose.block<3, 3>(0, 0) = -R_c_i * R_w_i.transpose();
  J_pose.block<3, 3>(0, 3) = R_c_i * SO3::hat(pt_i);
  return J_pose;
}

// Vec2d VIO::world2cam(Vec3d pt_c_)
// {

// }

Vec2d VIO::cameraPointToPixel(Vec3d pt_c) 
{
    return cameraPointToPixel(pt_c, 1.0);
}

Vec2d VIO::cameraPointToPixel(Vec3d pt_c, double scale_factor)
{
    // 检查点是否在相机前方
    if (pt_c.z() <= 0) {
        return Eigen::Vector2d(-1, -1); // 无效点
    }
    
    // 归一化平面坐标
    double x = pt_c.x() / pt_c.z();
    double y = pt_c.y() / pt_c.z();
    
    // 径向畸变参数 r
    double r2 = x*x + y*y;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    
    // 畸变校正（Brown-Conrady模型）
    double x_distorted = x * (1 + cam_d0_ * r2 + cam_d1_ * r4 + cam_d2_ * r6);
    double y_distorted = y * (1 + cam_d0_ * r2 + cam_d1_ * r4 + cam_d2_ * r6);
    
    // 切向畸变
    x_distorted += 2 * cam_d3_ * x * y + cam_d4_ * (r2 + 2 * x * x);
    y_distorted += cam_d3_ * (r2 + 2 * y * y) + 2 * cam_d4_ * x * y;
    
    // 像素坐标转换
    const double fx = cam_fx_ * scale_factor;
    const double fy = cam_fy_ * scale_factor;
    const double cx = cam_cx_ * scale_factor;
    const double cy = cam_cy_ * scale_factor;
    const double width = cam_width_ * scale_factor;
    const double height = cam_height_ * scale_factor;

    double u = fx * x_distorted + cx;
    double v = fy * y_distorted + cy;
    
    // 检查像素坐标是否在图像范围内
    if (u >= 0 && u < width && v >= 0 && v < height) {
        return Eigen::Vector2d(u, v);
    } else {
        return Eigen::Vector2d(-1, -1); // 超出图像范围
    }
}

bool VIO::isUVEffective(Vec2d pt_uv)
{
  if(pt_uv[0] < 0 || pt_uv[1] < 0){
    return false;
  }
  else{
    return true;
  }

}

void VIO::drawimg()
{
  //TODO:add if img != null
  if(new_frame_->pts.size()!=0)
  {
    int point_radius = 6;
    cv::Scalar point_color = cv::Scalar(0, 0, 255); 
    
    for (int i = 0; i < new_frame_->pts.size(); i++)
    {
      cv::Point2d cv_point(new_frame_->pts[i].px_ref[1], new_frame_->pts[i].px_ref[0]);
      circle(new_frame_->img_, cv_point, point_radius, point_color, -1);
    }
  }
}

void VIO::DirectPoseEstimationSingleLayer(   
  const cv::Mat &img1,
  const cv::Mat &img2,
  const VecVector2d &px_ref,
  std::vector<Vec3d> point_ref,
  Sophus::SE3 &T21) {

  const int iterations = 10;
  double cost = 0, lastCost = 0;
  auto t1 = chrono::steady_clock::now();
  JacobianAccumulator jaco_accu(img1, img2, px_ref, point_ref, T21);

  for (int iter = 0; iter < iterations; iter++) {
      jaco_accu.reset();
      cv::parallel_for_(cv::Range(0, px_ref.size()),
                        std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_accu));
      Mat6d H = jaco_accu.hessian();
      Vec6d b = jaco_accu.bias();

      // solve update and put it into estimation
      Vec6d update = H.ldlt().solve(b);;
      T21 = Sophus::SE3::exp(update) * T21;
      cost = jaco_accu.cost_func();

      if (std::isnan(update[0])) {
          // sometimes occurred when we have a black or white patch and H is irreversible
          cout << "update is nan" << endl;
          break;
      }
      if (iter > 0 && cost > lastCost) {
          cout << "cost increased: " << cost << ", " << lastCost << endl;
          break;
      }
      if (update.norm() < 1e-3) {
          // converge
          break;
      }

      lastCost = cost;
      cout << "iteration: " << iter << ", cost: " << cost << endl;
  }

  cout << "T21 = \n" << T21.matrix() << endl;
  auto t2 = chrono::steady_clock::now();
  auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  ROS_INFO("direct method for single layer: %f",time_used.count());

}

void VIO::DirectPoseEstimationMultiLayer()
{
  if (!last_frame_ || !new_frame_) {
    return;
  }
  if (last_frame_->pts.empty()) {
    return;
  }
  if (last_frame_->pyr_.empty()) {
    last_frame_->BuildPyramid(pyr_levels_);
  }
  if (new_frame_->pyr_.empty()) {
    new_frame_->BuildPyramid(pyr_levels_);
  }

  SE3 T_c_w_est = new_frame_->T_c_w;
  for (int level = pyr_levels_ - 1; level >= 0; --level) {
    const cv::Mat& ref_img = last_frame_->pyr_[level];
    const cv::Mat& cur_img = new_frame_->pyr_[level];
    const double scale_factor = 1.0 / static_cast<double>(1 << level);
    const double fx = cam_fx_ * scale_factor;
    const double fy = cam_fy_ * scale_factor;
    const double cx = cam_cx_ * scale_factor;
    const double cy = cam_cy_ * scale_factor;

    double last_cost = std::numeric_limits<double>::max();
    for (int iter = 0; iter < gn_iters_per_level_; ++iter) {
      Mat6d H = Mat6d::Zero();
      Vec6d b = Vec6d::Zero();
      double cost = 0.0;
      int valid_count = 0;

      for (const auto& pt : last_frame_->pts) {
        Vec3d pt_c = T_c_w_est * pt.pt_w;
        if (pt_c.z() <= 0) {
          continue;
        }

        const double X = pt_c.x();
        const double Y = pt_c.y();
        const double Z = pt_c.z();
        const double inv_Z = 1.0 / Z;
        const double inv_Z2 = inv_Z * inv_Z;
        const Vec2d px_ref = pt.px_ref * scale_factor;
        const Vec2d px_cur(fx * X * inv_Z + cx, fy * Y * inv_Z + cy);

        if (px_ref[0] < half_patch_size_ || px_ref[0] >= ref_img.cols - half_patch_size_ ||
            px_ref[1] < half_patch_size_ || px_ref[1] >= ref_img.rows - half_patch_size_ ||
            px_cur[0] < half_patch_size_ || px_cur[0] >= cur_img.cols - half_patch_size_ ||
            px_cur[1] < half_patch_size_ || px_cur[1] >= cur_img.rows - half_patch_size_) {
          continue;
        }

        for (const auto& offset : patch_pattern_) {
            const int dx = offset[0];
            const int dy = offset[1];
            const double residual =
                GetPixelValue(ref_img, px_ref[0] + dx, px_ref[1] + dy) -
                GetPixelValue(cur_img, px_cur[0] + dx, px_cur[1] + dy);

            Eigen::Vector2d J_img_pixel(
                0.5 * (GetPixelValue(cur_img, px_cur[0] + dx + 1, px_cur[1] + dy) -
                       GetPixelValue(cur_img, px_cur[0] + dx - 1, px_cur[1] + dy)),
                0.5 * (GetPixelValue(cur_img, px_cur[0] + dx, px_cur[1] + dy + 1) -
                       GetPixelValue(cur_img, px_cur[0] + dx, px_cur[1] + dy - 1)));

            Mat26d J_pixel_xi;
            J_pixel_xi(0, 0) = fx * inv_Z;
            J_pixel_xi(0, 1) = 0;
            J_pixel_xi(0, 2) = -fx * X * inv_Z2;
            J_pixel_xi(0, 3) = -fx * X * Y * inv_Z2;
            J_pixel_xi(0, 4) = fx + fx * X * X * inv_Z2;
            J_pixel_xi(0, 5) = -fx * Y * inv_Z;

            J_pixel_xi(1, 0) = 0;
            J_pixel_xi(1, 1) = fy * inv_Z;
            J_pixel_xi(1, 2) = -fy * Y * inv_Z2;
            J_pixel_xi(1, 3) = -fy - fy * Y * Y * inv_Z2;
            J_pixel_xi(1, 4) = fy * X * Y * inv_Z2;
            J_pixel_xi(1, 5) = fy * X * inv_Z;

            const Vec6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();
            H += J * J.transpose();
            b += -residual * J;
            cost += residual * residual;
            valid_count++;
        }
      }

      if (valid_count < 10) {
        break;
      }

      Vec6d update = H.ldlt().solve(b);
      if (std::isnan(update[0])) {
        break;
      }
      T_c_w_est = Sophus::SE3::exp(update) * T_c_w_est;
      if (cost > last_cost || update.norm() < 1e-3) {
        break;
      }
      last_cost = cost;
    }
  }

  new_frame_->T_c_w = T_c_w_est;
}
