#include "merbots_grasp_srv/grasp.hpp"

Grasp::Grasp(ros::NodeHandle nh){
  nh_=nh;
  initialized_=false;
  joint_msg_.name.push_back("Slew");
  joint_msg_.name.push_back("Shoulder");
  joint_msg_.name.push_back("Elbow");
  joint_msg_.name.push_back("JawRotate");
  joint_msg_.name.push_back("JawOpening");
  for(int i=0; i<4; i++){
    joint_msg_.velocity.push_back(0.0);
  }
  joint_msg_.velocity.push_back(-0.3);


   service_=nh.advertiseService("/doGrasping", &Grasp::doGrasping, this);
}

Grasp::~Grasp(){}

void Grasp::jointsCallback(const sensor_msgs::JointStateConstPtr &msg){
  effort_=msg->effort[0];
  gripper_pose_=msg->position[4];
  initialized_=true;
}

bool Grasp::doGrasping(merbots_grasp_srv::grasp_srv::Request &req, merbots_grasp_srv::grasp_srv::Response &res){
  initialized_=false;
  joint_sub_=nh_.subscribe<sensor_msgs::JointState>(req.joint_state_topic, 1, &Grasp::jointsCallback, this);
  joint_pub_=nh_.advertise<sensor_msgs::JointState>(req.command_topic, 1);
  while(ros::ok() && !initialized_){
    usleep(1000);
    ros::spinOnce();
  }
  while(effort_<req.grasped_current && gripper_pose_>=0.05 && ros::ok()){
    joint_msg_.header.stamp=ros::Time::now();
    joint_pub_.publish(joint_msg_);
    ros::spinOnce();
    usleep(1000);
  }
  if(effort_>=req.grasped_current){
    res.success=true;
  }
  else{
    joint_msg_.velocity[4]=0.3;
    while(effort_<1.2 && gripper_pose_<=1.3 && ros::ok()){
      joint_msg_.header.stamp=ros::Time::now();
      joint_pub_.publish(joint_msg_);
      ros::spinOnce();
      usleep(1000);
    }
    joint_msg_.velocity[4]=-0.3;
    res.success=false;
  }
  return true;
}
