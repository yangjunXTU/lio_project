/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-10-22 08:10:37
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-10-29 09:18:35
 * @FilePath: /lio_project/include/utils/vio.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef H_VIO_HPP
#define H_VIO_HPP

#include "utils/frame.h"
#include <ros/ros.h>
#include "utils/iekf.h"
#include "utils/nav_state.h"
#include "utils/eigen_types.h"
#include "utils/math_utils.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "utils/point_types.h"
#include <opencv2/opencv.hpp>
#include "utils/direct_method.h"

struct CameraConfig {
    std::string model = "pinhole";
    int width = 0;
    int height = 0;
    double scale = 1.0;
    Mat3d K = Mat3d::Identity();
    std::vector<double> D;
};

struct VioConfig {
    int pyr_levels = 3;
    int half_patch_size = 1;
    int max_points = 400;
    int gn_iters_per_level = 5;
    int min_valid_residuals = 80;
    int grid_rows = 6;
    int grid_cols = 8;
    int max_points_per_cell = 2;
    double min_gradient = 10.0;
    double min_depth = 1.0;
    double max_depth = 30.0;
    double huber_delta = 10.0;
    double info_scale = 0.01;
    double min_inlier_ratio = 0.6;
    double max_mean_residual = 12.0;
    double max_update_translation = 0.3;
    double max_update_rotation_deg = 8.0;
    double min_keyframe_translation = 0.15;
    double min_keyframe_rotation_deg = 5.0;
};

struct VisualTrackingStats {
    int candidate_points = 0;
    int selected_points = 0;
    int total_residuals = 0;
    int valid_residuals = 0;
    int inlier_residuals = 0;
    double mean_cost = 0.0;
    double mean_abs_residual = 0.0;
    double inlier_ratio = 0.0;
    double update_translation_norm = 0.0;
    double update_rotation_deg = 0.0;
    double reference_translation_norm = 0.0;
    double reference_rotation_deg = 0.0;
    bool update_accepted = false;
    bool promote_reference = false;
};

class VIO
{
private:
    /* data */
public:
    FramePtr new_frame_;
    FramePtr last_frame_;
    cv::Mat img_cp, img_rgb;
    NavStated *nav_state_w_i_ptr_;
    CloudPtr vio_scan_world{new PointCloudType};
    bool flg_first_scan_vio = true;
    bool visual_measurement_ready_ = false;

    Mat3d R_l_i, R_c_i, R_c_l, R_c_w;
    Vec3d t_l_i, t_c_i, t_c_l, t_c_w;
    SE3 T_c_i;
    int pyr_levels_ = 3;
    int half_patch_size_ = 1;
    int max_visual_points_ = 400;
    int gn_iters_per_level_ = 5;
    int min_valid_residuals_ = 80;
    int grid_rows_ = 6;
    int grid_cols_ = 8;
    int max_points_per_cell_ = 2;
    double min_gradient_ = 10.0;
    double min_depth_ = 1.0;
    double max_depth_ = 30.0;
    double huber_delta_ = 10.0;
    double info_scale_ = 0.01;
    double min_inlier_ratio_ = 0.6;
    double max_mean_residual_ = 12.0;
    double max_update_translation_ = 0.3;
    double max_update_rotation_deg_ = 8.0;
    double min_keyframe_translation_ = 0.15;
    double min_keyframe_rotation_deg_ = 5.0;
    std::vector<Vec2i> patch_pattern_;
    VisualTrackingStats tracking_stats_;


    std::string cam_model_;
    double cam_width_,cam_height_,scale_;
    double cam_fx_,cam_fy_,cam_cx_,cam_cy_;
    double cam_d0_,cam_d1_,cam_d2_,cam_d3_,cam_d4_;

    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    
    VIO(/* args */);
    ~VIO();
    void processFrame(cv::Mat &img, SE3 &T_w_i_meas);
    void setImuToLidarExtrinsic(const Vec3d &transl, const Mat3d &rot);
    void setLidarToCameraExtrinsic(const Mat3d &R_c_l, const Vec3d &t_c_l);
    void initializeVIO();
    void setCameraParameter(const CameraConfig& camera_config);
    void setVioParameter(const VioConfig& vio_config);
    
    void setCameraPoseFromNavState(NavStated nav_state_w_i);
    const VisualTrackingStats& trackingStats() const { return tracking_stats_; }
    void resetTrackingStats();
    void finalizeFrame();
    void buildPatchPattern();
    void publishStaticTransformC2I();
    void buildVisibleMapPoints(Frame &new_frame);
    void stateEstimate();
    bool ShouldAcceptVisualUpdate() const;
    bool ShouldPromoteReferenceFrame() const;
    double ComputeReferenceMotionTranslation() const;
    double ComputeReferenceMotionRotationDeg() const;
    double ScoreVisualPoint(const Vec3d& pt_c, double gradient) const;
    bool IsPointPhotometricallyValid(const Frame& frame, const Vec3d& pt_c, const Vec2d& px, int margin) const;
    Eigen::Vector2d ComputeImageGradient(const cv::Mat& img, double u, double v) const;
    Eigen::Matrix<double, 2, 3> ComputeProjectionJacobian(const Vec3d& pt_c, double scale_factor) const;
    Eigen::Matrix<double, 3, 6> ComputePointJacobianWrtPose(const SE3& T_w_i, const Vec3d& pt_w) const;
    void ComputeResidualAndJacobians(const SE3 &T_w_i, Mat18d &HTVH, Vec18d &HTVr);
    Vec2d cameraPointToPixel(Vec3d pt_c);
    Vec2d cameraPointToPixel(Vec3d pt_c, double scale_factor);
    bool isUVEffective(Vec2d pt_uv);
    void drawimg();
    void DirectPoseEstimationSingleLayer(const cv::Mat &img1,
                                        const cv::Mat &img2,
                                        const VecVector2d &px_ref,
                                        std::vector<Vec3d> point_ref,
                                        Sophus::SE3 &T21);
    void DirectPoseEstimationMultiLayer();
};



typedef std::shared_ptr<VIO> VIOPtr;

#endif
