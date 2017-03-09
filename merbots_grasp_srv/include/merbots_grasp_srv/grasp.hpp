#ifndef GRASP_HPP
#define GRASP_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "merbots_grasp_srv/grasp_srv.h"


class Grasp{
  void jointsCallback(const sensor_msgs::JointStateConstPtr &msg);
   bool doGrasping(merbots_grasp_srv::grasp_srv::Request &req, merbots_grasp_srv::grasp_srv::Response &res);
  ros::NodeHandle nh_;
  ros::Subscriber joint_sub_;
  ros::Publisher  joint_pub_;
  sensor_msgs::JointState joint_msg_;
  float effort_;
  bool initialized_;
  float gripper_pose_;
  ros::ServiceServer service_;

public:
  Grasp(ros::NodeHandle nh);
  ~Grasp();

};

#endif // GRASP_HPP
