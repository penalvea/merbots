#ifndef PI_CONTROLLER_HPP
#define PI_CONTROLLER_HPP

#include <ros/ros.h>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>


class PIController{
  std::vector<float> p_values_, i_values_;
  std::vector<float> error_accum_;
   std::vector<float> error_;
  std::vector<float> last_vels_, last_joints_, last_odom_;
  Eigen::Transform<float, 3, Eigen::Affine> wMlv_;
  ros::Time last_time_;
  bool init_;

public:
  PIController(std::vector<float> p_values, std::vector<float> i_values);
  ~PIController();
  Eigen::MatrixXd getVels(Eigen::MatrixXd vels, ros::Time time);
  void updateController(std::vector<float> odom, std::vector<float> current_joints, ros::Time time);
};
typedef boost::shared_ptr<PIController> PIControllerPtr;

#endif // PI_CONTROLLER_HPP
