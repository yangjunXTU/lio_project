/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-10-22 07:41:16
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-10-22 08:15:58
 * @FilePath: /lio_project/src/frame.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "utils/frame.h"

int Frame::frame_counter_ = 0;

Frame::Frame(const cv::Mat &img)
{
    img_ = img;
    id_ = ++frame_counter_;

}

Frame::~Frame()
{
}

void Frame::BuildPyramid(int levels)
{
    pyr_.clear();
    if (img_.empty() || levels <= 0) {
        return;
    }

    pyr_.push_back(img_);
    for (int level = 1; level < levels; ++level) {
        cv::Mat downsampled;
        cv::pyrDown(pyr_.back(), downsampled);
        pyr_.push_back(downsampled);
    }
}
