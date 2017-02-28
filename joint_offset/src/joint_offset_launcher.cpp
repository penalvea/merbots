/**
 * This program launches the joint_offset.
 *  Created on: 27/02/2017
 *
 */
#include <ros/ros.h>
#include <grasper/joint_offset.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_listener.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_offset_launcher");
  ros::NodeHandle nh;

  std::string joint_state_command("/uwsim/joint_state_command");
  nh.getParam("joint_state_command", joint_state_command);

  std::string joint_state("/uwsim/joint_state");
  nh.getParam("joint_state", joint_state);

  std::string joint_state_fixed("/uwsim/joint_state_fixed");
  nh.getParam("joint_state_fixed", joint_state_fixed);

  std::string target_frame_id("asdf");
  nh.getParam("target_frame_id", target_frame_id);

  std::string source_frame_id("/uwsim/joint_state_fixed");
  nh.getParam("source_frame_id", source_frame_id);


  JointOffset* joint_offset = new JointOffset(nh, joint_state, joint_state_command, joint_state_fixed);







  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/marker_filter_node/marker_pose",1);

  tf::TransformListener *listener_;

  ros::Rate r(1);
  while (ros::ok()) {

    tf::StampedTransform transformTF;
    try{
      //listener.waitForTransform(destination_frame, original_frame, ros::Time(0), ros::Duration(10.0) );
      listener_->lookupTransform(target_frame_id, source_frame_id, ros::Time::now() - ros::Duration(0.1), transformTF);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Cannot find TRANSFORM %s",ex.what());
      return false;
    }

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = target_frame_id;
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = transformTF.getOrigin().x();
    target_pose.pose.position.y = transformTF.getOrigin().y();
    target_pose.pose.position.z = transformTF.getOrigin().z();
    target_pose.pose.orientation.x = transformTF.getRotation().x();
    target_pose.pose.orientation.y = transformTF.getRotation().y();
    target_pose.pose.orientation.z = transformTF.getRotation().z();
    target_pose.pose.orientation.w = transformTF.getRotation().w();

    pub.publish(target_pose);

    r.sleep();
    ros::spinOnce();
  }


  ros::spin();

  return (0);
}
