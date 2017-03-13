#ifndef GRASP_HPP
#define GRASP_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "merbots_grasp_srv/grasp_srv.h"
#include "merbots_grasp_srv/open_gripper_srv.h"


class Grasp{
  void jointsCallback(const sensor_msgs::JointStateConstPtr &msg);
  void jointsCallbackOpening(const sensor_msgs::JointStateConstPtr &msg);

  bool doGrasping(merbots_grasp_srv::grasp_srv::Request &req, merbots_grasp_srv::grasp_srv::Response &res);
  bool doOpening(merbots_grasp_srv::open_gripper_srv::Request &req, merbots_grasp_srv::open_gripper_srv::Response &res);
  ros::NodeHandle nh_;
  ros::Subscriber joint_sub_;
  ros::Publisher  joint_pub_;
  sensor_msgs::JointState joint_msg_;
  ros::Subscriber joint_sub_opening_;
  ros::Publisher  joint_pub_opening_;

  float effort_;
  bool initialized_;
  float gripper_pose_;
  ros::ServiceServer service_;

  float effort_opening_;
  bool initialized_opening_;
  float gripper_pose_opening_;
  ros::ServiceServer service_opening_;

public:
  Grasp(ros::NodeHandle nh);
  ~Grasp();

};

#endif // GRASP_HPP
