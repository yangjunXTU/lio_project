#include "lio_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lio_project_node");
  

  LIO * fast_lio_instance = new LIO();
  ros::NodeHandle nh = fast_lio_instance->nh;
  ros::Rate rate(30);
  bool status = ros::ok();
  ros::spin();


}