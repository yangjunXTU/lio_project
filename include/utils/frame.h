/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-10-22 07:40:51
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-10-29 09:20:45
 * @FilePath: /lio_project/include/utils/frame.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef H_FRAME_HPP
#define H_FRAME_HPP


#include <boost/noncopyable.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include "utils/eigen_types.h"
#include <vector>

struct PTs_3d2
{
  Vec3d pt_w;
  Vec2d px_ref;
};


class Frame : boost::noncopyable
{
private:
    /* data */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int frame_counter_; 
    int id_;  
    SE3 T_c_w;
    SE3 T_w_i_meas;
    std::vector<PTs_3d2> pts;
    std::vector<cv::Mat> pyr_;
    
    cv::Mat img_;

    Frame(const cv::Mat &img);
    ~Frame();
    void BuildPyramid(int levels);

    /// Transform a world point into the current camera frame.
    inline Vec3d worldToCamera(const Vec3d &xyz_w) const { return T_c_w * xyz_w; }
};

// typedef std::unique_ptr<Frame> FramePtr;
typedef std::shared_ptr<Frame> FramePtr;

#endif
