#include <ros/ros.h>
#include "merbots_grasp_srv/grasp.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "grasp_service");
  ros::NodeHandle nh;

  Grasp grasp(nh);
  ros::spin();
  return 0;
}
