#ifndef GOAL_HPP
#define GOAL_HPP
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <task_priority/Error_msg.h>
#include <merbots_grasp_srv/grasp_srv.h>
#include <merbots_grasp_srv/grasp_station_srv.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <std_msgs/String.h>



class Goal{
  bool initialized_;
  float max_cartesian_vel_;
  Eigen::MatrixXd cartesian_offset_;
public:
  Goal();
  virtual ~Goal();
  virtual Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom)=0;
  virtual task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask)=0;
  Eigen::Vector3d quaternionsSubstraction(Eigen::Quaterniond quat_desired, Eigen::Quaterniond quat_current);
  void setInitialized(bool initialized);
  bool getInitialized();
  void setMaxCartesianVel(float max_cartesian_vel);
  Eigen::MatrixXd limitCaresianVel(Eigen::MatrixXd vels);
  void incrementOffset(Eigen::MatrixXd cartesian_offset);
  Eigen::MatrixXd getCartesianOffset();
};
typedef boost::shared_ptr<Goal> GoalPtr;


class GoalFixedPose: public Goal{
  KDL::ChainFkSolverPos_recursive *fk_chain_;
  std::vector<int> mask_cart_;
  Eigen::MatrixXd goal_;
  std::vector<int> joints_relation_;
  Eigen::MatrixXd last_cartesian_vel_;
public:
  GoalFixedPose(Eigen::MatrixXd goal, KDL::Chain chain, std::vector<int> mask_cart, std::vector<int> joints_relation, float max_cartesian_vel);
  ~GoalFixedPose();
  Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom);
  task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask);
};
typedef boost::shared_ptr<GoalFixedPose> GoalFixedPosePtr;

class GoalROSPose: public Goal{

  KDL::ChainFkSolverPos_recursive *fk_chain_;
  std::vector<int> mask_cart_;
  geometry_msgs::Pose goal_;
  std::vector<int> joints_relation_;
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
   Eigen::MatrixXd last_cartesian_vel_;
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
public:
  GoalROSPose(KDL::Chain chain, std::vector<int> mask_cart, std::string pose_topic, ros::NodeHandle &nh, std::vector<int> joints_relation, float max_cartesian_vel);
  ~GoalROSPose();
  Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom);
  task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask);
};
typedef boost::shared_ptr<GoalROSPose> GoalROSPosePtr;


class GoalROSTwist: public Goal{

  std::vector<int> mask_cart_;
  geometry_msgs::Twist goal_;
  ros::NodeHandle nh_;
  ros::Subscriber twist_sub_;
  Eigen::MatrixXd last_cartesian_vel_;
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
public:
  GoalROSTwist(std::vector<int> mask_cart, std::string twist_topic, ros::NodeHandle &nh, float max_cartesian_vel);
  ~GoalROSTwist();
  Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom);
  task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask);
};
typedef boost::shared_ptr<GoalROSTwist> GoalROSTwistPtr;

class GoalJointsPosition: public Goal{
  std::vector<float> joints_position_;
   Eigen::MatrixXd last_joint_vel_;
public:
  GoalJointsPosition(std::vector<float> joints_position);
  ~GoalJointsPosition();
  Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom);
  task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask);
};
typedef boost::shared_ptr<GoalJointsPosition> GoalJointsPositionPtr;

class GoalGrasp: public Goal{
  KDL::ChainFkSolverPos_recursive *fk_chain_;
  std::vector<int> mask_cart_;
  geometry_msgs::Pose goal_;
  std::vector<int> joints_relation_;
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  Eigen::MatrixXd last_cartesian_vel_;
  int step_;
  ros::ServiceClient grasp_client_, enable_keep_position_, disable_keep_position_;
  bool pose_received_;
  int pose_reached_;
  std::string joint_state_topic_, command_joint_topic_;
  ros::Subscriber force_sub_;
  float max_force_;
  float current_force_;
  bool force_detected_, last_detection_;
  int force_direction_;
  ros::ServiceClient force_set_zero_;

  // Force offset vars
  float force_offfset_torque_y_, force_offfset_torque_z_;
  bool reset_offset_next_callback_;


  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
public:
  GoalGrasp(KDL::Chain chain, std::vector<int> mask_cart, std::string pose_topic, ros::NodeHandle &nh, std::vector<int> joints_relation, float max_cartesian_vel, std::string joint_state_topic, std::string command_joint_topic, bool force_sensor, std::string force_sensor_topic, float max_force, std::string force_set_zero);
  ~GoalGrasp();
  Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom);
  task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask);
};
typedef boost::shared_ptr<GoalGrasp> GoalGraspPtr;


class GoalROSJointsState: public Goal{
  std::vector<float> joints_position_;
  Eigen::MatrixXd last_joint_vel_;
  ros::Subscriber joint_state_sub_;
  ros::NodeHandle nh_;
  ros::Publisher finished_pub_;
  ros::Subscriber grasping_sub_;
  int grasping_;
  ros::ServiceClient grasp_client_;

  void graspingCallback(const std_msgs::String::ConstPtr &msg);
  void jointsCallback(const sensor_msgs::JointState::ConstPtr &msg);
public:
  GoalROSJointsState(std::string topic_name, ros::NodeHandle nh);
  ~GoalROSJointsState();
  Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom);
  task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask);
};
typedef boost::shared_ptr<GoalROSJointsState> GoalROSJointsStatePtr;




#endif // GOAL_HPP
