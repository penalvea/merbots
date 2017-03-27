#ifndef GRASP_HPP
#define GRASP_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "merbots_grasp_srv/grasp_srv.h"
#include "merbots_grasp_srv/open_gripper_srv.h"
#include "merbots_grasp_srv/grasp_station_srv.h"
#include <auv_msgs/WorldWaypointReq.h>
#include<geometry_msgs/WrenchStamped.h>


class Grasp{
  void jointsCallback(const sensor_msgs::JointStateConstPtr &msg);
  void jointsCallbackOpening(const sensor_msgs::JointStateConstPtr &msg);
  void jointsCallbackGraspStation(const sensor_msgs::JointStateConstPtr &msg);

  bool doGrasping(merbots_grasp_srv::grasp_srv::Request &req, merbots_grasp_srv::grasp_srv::Response &res);
  bool doOpening(merbots_grasp_srv::open_gripper_srv::Request &req, merbots_grasp_srv::open_gripper_srv::Response &res);
  bool doGraspingStation(merbots_grasp_srv::grasp_station_srv::Request &req, merbots_grasp_srv::grasp_station_srv::Response &res);


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

  float effort_station_;
  bool initialized_station_;
  float gripper_pose_station_;
  ros::ServiceServer service_station_;
  ros::Publisher station_pub_;


  ros::Subscriber sensor0_sub_, sensor1_sub_, sensor2_sub_, sensor3_sub_;
  void sensor0Callback(const geometry_msgs::WrenchStampedConstPtr &msg);
  void sensor1Callback(const geometry_msgs::WrenchStampedConstPtr &msg);
  void sensor2Callback(const geometry_msgs::WrenchStampedConstPtr &msg);
  void sensor3Callback(const geometry_msgs::WrenchStampedConstPtr &msg);

  float sensor0_initial_, sensor1_initial_, sensor2_initial_, sensor3_initial_;
  float sensor0_, sensor1_, sensor2_, sensor3_;
  bool isGrasped();



public:
  Grasp(ros::NodeHandle nh);
  ~Grasp();

};

#endif // GRASP_HPP
