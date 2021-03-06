#ifndef GRASPCONTROLLER_HPP
#define GRASPCONTROLLER_HPP

#include "task_priority/task.hpp"
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>
#include "task_priority/pi_controller.hpp"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>


class GraspController{

  std::vector<MultiTaskPtr> multitasks_;
  std::vector<float> max_joint_limit_, min_joint_limit_;
  std::vector<std::vector<float> > max_cartesian_limits_, min_cartesian_limits_;
  float max_cartesian_vel_;
  int n_joints_;
  float acceleration_;
  float max_joint_vel_;
  float sampling_duration_;
  ros::NodeHandle nh_;
  ros::Publisher joints_pub_, vehicle_pub_, status_pub_, effector_twist_, vehicle_twist_, effector_pose_, vehicle_pose_, constraints_pub_;
  ros::Subscriber joints_sub_;
  tf::TransformListener listener;
  std::vector<float> current_joints_;
  bool joints_init_, goal_init_;
  std::string world_tf_, vehicle_tf_;
  std::vector<KDL::Chain> chains_;
  std::vector<std::vector<int> > chain_joint_relations_;
  float sensor_threshold_value_;
  bool simulation_;
  PIControllerPtr pi_controller_;


  void jointsCallback(const sensor_msgs::JointStateConstPtr  &msg);
  void publishVels(Eigen::MatrixXd vels);
  Eigen::MatrixXd limitVels(Eigen::MatrixXd vels);
  float calculateMaxNegativeVel(float current_joint, float min_joint_value, float acceleration, float sampling_duration);
  float calculateMaxPositiveVel(float current_joint, float max_joint_value, float acceleration, float sampling_duration);
  float calculateMaxNegativeVel(float difference, float acceleration, float sampling_duration);
  float calculateMaxPositiveVel(float difference, float acceleration, float sampling_duration);
  Eigen::Vector3d quaternionsSubstraction(Eigen::Quaterniond quat_desired, Eigen::Quaterniond quat_current);
  std::vector<std::vector<std::vector<float> > > calculateMaxCartesianVels(std::vector<float> joints, std::vector<float> odom);
  void publishStatus(Eigen::MatrixXd vels);
  void publishTwist(Eigen::MatrixXd vels, std::vector<float> odom);
  void publishPose( std::vector<float> odom);
  void publishJoints( std::vector<float> joints);
  void publishConstraints();
  bool isInsideLimits(std::vector<float> odom);
public:
  GraspController(std::vector<MultiTaskPtr> multitasks, int n_joints, std::vector<float> max_joint_limit, std::vector<float> min_joint_limit, std::vector<std::vector<float> > max_cartesian_limits, std::vector<std::vector<float> > min_cartesian_limits, float max_cartesian_vel, float acceleration, float max_joint_vel, float sampling_duration, ros::NodeHandle nh, std::string arm_joint_state_topic, std::string arm_joint_command_topic, std::string vehicle_tf, std::string world_tf, std::string vehicle_command_topic, std::vector<KDL::Chain> chains, std::vector<std::vector<int> > chain_joint_relations, bool simulation, std::vector<float> p_values, std::vector<float> i_values, std::vector<float> d_values);
  ~GraspController();
  std::vector<float> getSolution(std::vector<float> joints, std::vector<float> odom, std::vector<int> important_tasks);


};
typedef boost::shared_ptr<GraspController> GraspControllerPtr;

#endif // GRASPCONTROLLER_HPP
