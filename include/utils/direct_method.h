/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-10-28 06:41:54
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-10-29 02:40:07
 * @FilePath: /lio_project/include/utils/direct_method.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <opencv2/opencv.hpp>
// #include <sophus/se3.h>
#include "utils/eigen_types.h"
#include <boost/format.hpp>

using namespace std;
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

class JacobianAccumulator
{
private:
    const cv::Mat &img1;
    const cv::Mat &img2;
    const VecVector2d &px_ref;
    const std::vector<Vec3d> point_ref;
    Sophus::SE3 &T21;
    VecVector2d projection; // projected points

    std::mutex hessian_mutex;
    Mat6d H = Mat6d::Zero();
    Vec6d b = Vec6d::Zero();
    double cost = 0;
    double fx = 635.221, fy = 636.414, cx = 635.881, cy = 358.556;

public:
    JacobianAccumulator(const cv::Mat &img1_,
                        const cv::Mat &img2_,
                        const VecVector2d &px_ref_,
                        const std::vector<Vec3d> point_ref_,
                        Sophus::SE3 &T21_);
    ~JacobianAccumulator();

        /// accumulate jacobians in a range
    void accumulate_jacobian();

    /// get hessian matrix
    Mat6d hessian() const { return H; }

    /// get bias
    Vec6d bias() const { return b; }

    /// get total cost
    double cost_func() const { return cost; }

    VecVector2d projected_points() const { return projection; }

    void reset() {
        H = Mat6d::Zero();
        b = Vec6d::Zero();
        cost = 0;
    }

    inline float GetPixelValue(const cv::Mat &img, float x, float y) {
        // boundary check
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
};

