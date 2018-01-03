#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  std::ofstream file2;
  file2.open("world_to_effector.txt");
  while (node.ok()){
    tf::StampedTransform transform;
    usleep(100000);
    try{
      //listener.lookupTransform("/world", "/arm5/end_effector",ros::Time(0), transform);
      geometry_msgs::Twist twist;
      ros::Time time=ros::Time(0);
      listener.lookupTwist( "/world", "/girona500/end_effector", time,  ros::Duration(0.1), twist);

      //file2<<transform.stamp_.sec<<" "<<transform.stamp_.nsec<<" "<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<<" "<<transform.getRotation().x()<<" "<<transform.getRotation().y()<<" "<<transform.getRotation().z()<<" "<<transform.getRotation().w()<<std::endl;
      file2<<time.sec<<" "<<time.nsec<<" "<<twist.linear.x<<" "<<twist.linear.y<<" "<<twist.linear.z<<" "<<twist.angular.x<<" "<<twist.angular.y<<" "<<twist.angular.z<<std::endl;


    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
std::cout<<"No Leido-------------------"<<std::endl;
    }

  }
  file2.close();
  return 0;
}
