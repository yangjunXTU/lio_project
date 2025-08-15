/*
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-08-12 02:03:20
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-08-12 07:26:30
 * @FilePath: /lio_project_wk/src/lio_project/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "lio_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lio_project_node");
  

  LIO * fast_lio_instance = new LIO();
  // ros::NodeHandle nh = fast_lio_instance->nh;
  fast_lio_instance->run();
  ros::Rate rate(30);
  bool status = ros::ok();
  ros::spin();


}