#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/SVD>
#include <Eigen/Dense>



int main(int argc, char** argv){
  ros::init(argc, argv, "markerToGrasp");
  ros::NodeHandle nh;
  ros::Publisher grasp_pub=nh.advertise<geometry_msgs::Pose>("final_grasp",1);
  tf::TransformListener listener;
  Eigen::Affine3d rz =
        Eigen::Affine3d(Eigen::AngleAxisd(-1.57, Eigen::Vector3d(0, 0, 1)));
  Eigen::Affine3d ry =
        Eigen::Affine3d(Eigen::AngleAxisd(3.14, Eigen::Vector3d(0, 1, 0)));

  Eigen::Affine3d transz(Eigen::Translation3d(Eigen::Vector3d(0,0,0.25)));
  Eigen::Affine3d wMm;

  while(ros::ok()){
    ros::spinOnce();
     try{
    tf::StampedTransform transform;
    listener.lookupTransform("world", "amphora_marker", ros::Time(0), transform);

    tf::poseTFToEigen(transform, wMm);
    Eigen::Affine3d wMg=wMm*transz*rz*ry;
    std::cout<<"wMg"<<std::endl;
    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        std::cout<<wMg(i,j)<<" ";
      }
      std::cout<<std::endl;
    }

    tf::Transform transform2;
    tf::poseEigenToTF(wMg, transform2);
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "final_grasp"));
    geometry_msgs::Pose pose_msg;
    tf::poseTFToMsg(transform2, pose_msg);
    grasp_pub.publish(pose_msg);
    usleep(10000);
    }
    catch(tf::TransformException ex){
      ROS_ERROR("%s\n", ex.what());
      usleep(10000);
    }
  }
}
