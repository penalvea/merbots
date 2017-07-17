#include <task_priority/goal.hpp>

Eigen::MatrixXd pinvMat(Eigen::MatrixXd matrix){
  double epsilon=std::numeric_limits<double>::epsilon();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance=epsilon*std::max(matrix.cols(), matrix.rows())*svd.singularValues().array().abs()(0);
  return svd.matrixV()*(svd.singularValues().array().abs()>tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal()*svd.matrixU().adjoint();

}
Goal::Goal(){
  initialized_=false;
  cartesian_offset_.resize(3,1);
  cartesian_offset_.setZero();
}
Goal::~Goal(){}

Eigen::Vector3d Goal::quaternionsSubstraction(Eigen::Quaterniond quat_desired, Eigen::Quaterniond quat_current){
  if(quat_current.w()<0){
    quat_current.x()=-1*quat_current.x();
    quat_current.y()=-1*quat_current.y();
    quat_current.z()=-1*quat_current.z();
    quat_current.w()=-1*quat_current.w();
  }

  if(quat_desired.w()<0){
      quat_desired.x()=-1*quat_current.x();
      quat_desired.y()=-1*quat_current.y();
      quat_desired.z()=-1*quat_current.z();
      quat_desired.w()=-1*quat_current.w();
    }
  Eigen::Vector3d v1(quat_desired.x(), quat_desired.y(), quat_desired.z());
  Eigen::Vector3d v1_aux(quat_desired.x(), quat_desired.y(), quat_desired.z());
  Eigen::Vector3d v2(quat_current.x(), quat_current.y(), quat_current.z());
  Eigen::Vector3d v2_aux(quat_current.x(), quat_current.y(), quat_current.z());
  v1_aux.cross(v2);
  double norm=v1_aux.norm();
  double angle=std::atan2(norm, v1.adjoint()*v2);
  if(angle>(M_PI/2)+0.000000000001){

    quat_desired.x()=-quat_desired.x();
    quat_desired.y()=-quat_desired.y();
    quat_desired.z()=-quat_desired.z();
    quat_desired.w()=-quat_desired.w();
  }

  return quat_current.w()*v1-quat_desired.w()*v2+v2_aux.cross(v1);


  /*tf::Quaternion quat1(quat_desired.x(), quat_desired.y(), quat_desired.z(), quat_desired.w());
  tf::Matrix3x3 mat1(quat1);
  double roll_desired, pitch_desired, yaw_desired;
  mat1.getRPY(roll_desired, pitch_desired, yaw_desired);

  tf::Quaternion quat2(quat_current.x(), quat_current.y(), quat_current.z(), quat_current.w());
  tf::Matrix3x3 mat2(quat2);
  double roll_current, pitch_current, yaw_current;
  mat2.getRPY(roll_current, pitch_current, yaw_current);



  Eigen::Vector3d diff;
  diff[0]=(roll_desired-roll_current)+2*M_PI*std::floor((M_PI-(roll_desired-roll_current))/(2*M_PI));
  diff[1]=(pitch_desired-pitch_current)+2*M_PI*std::floor((M_PI-(pitch_desired-pitch_current))/(2*M_PI));
  diff[2]=(yaw_desired-yaw_current)+2*M_PI*std::floor((M_PI-(yaw_desired-yaw_current))/(2*M_PI));
  return diff;*/

}
void Goal::setMaxCartesianVel(float max_cartesian_vel){
  max_cartesian_vel_=max_cartesian_vel;
}

void Goal::setMaxJointVel(float max_joint_vel){
  max_joint_vel_=max_joint_vel;
}

Eigen::MatrixXd Goal::limitCartesianVel(Eigen::MatrixXd vels){
  float max=0;
  for(int i=0; i<vels.rows(); i++){
    if(std::abs(vels(i,0))>max){
      max=std::abs(vels(i,0));
    }
  }
  if(max>max_cartesian_vel_){
    for(int i=0; i<vels.rows(); i++){
      vels(i,0)=vels(i,0)*max_cartesian_vel_/max;
    }
  }
  return vels;
}

Eigen::MatrixXd Goal::limitJointVel(Eigen::MatrixXd vels){
  float max=0;
  for(int i=0; i<vels.rows(); i++){
    if(std::abs(vels(i,0))>max){
      max=std::abs(vels(i,0));
    }
  }
  std::cout<<"max vel="<<max<<std::endl;
  std::cout<<"max_joint_vel="<<max_joint_vel_<<std::endl;
  if(max>max_joint_vel_){
    for(int i=0; i<vels.rows(); i++){
      vels(i,0)=vels(i,0)*max_joint_vel_/max;
    }
  }
  return vels;
}

void Goal::setInitialized(bool initialized){
  initialized_=initialized;
}
bool Goal::getInitialized(){
  return initialized_;
}
void Goal::incrementOffset(Eigen::MatrixXd cartesian_offset){
  cartesian_offset_=cartesian_offset_+cartesian_offset;
}
Eigen::MatrixXd Goal::getCartesianOffset(){
  return cartesian_offset_;
}



GoalFixedPose::GoalFixedPose(Eigen::MatrixXd goal, KDL::Chain chain, std::vector<int> mask_cart, std::vector<int> joints_relation, float max_cartesian_vel, float max_joint_vel):Goal(){
  KDL::Chain chain_odom;
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
  chain_odom.addChain(chain);

  fk_chain_=new KDL::ChainFkSolverPos_recursive(chain_odom);
  mask_cart_=mask_cart;
  goal_=goal;
  joints_relation_=joints_relation;
  setInitialized(true);
  setMaxCartesianVel(max_cartesian_vel);
  setMaxJointVel(max_joint_vel);
}

GoalFixedPose::~GoalFixedPose(){}


Eigen::MatrixXd GoalFixedPose::getGoal(std::vector<float> joints, std::vector<float> odom){
  KDL::JntArray jq(odom.size()+joints_relation_.size());
  for(int i=0; i<odom.size(); i++){
    jq(i)=odom[i];
  }
  for(int i=0; i<joints_relation_.size(); i++){
    jq(i+odom.size())=joints[joints_relation_[i]];
  }

  KDL::Frame cartpos;
  fk_chain_->JntToCart(jq, cartpos);
  double x,y,z,w;
  cartpos.M.GetQuaternion(x,y,z,w);
  Eigen::Quaterniond quat_current(w,x,y,z);
  Eigen::Matrix3d current;
  current=Eigen::AngleAxisd(goal_(5,0), Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(goal_(4,0), Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(goal_(3,0), Eigen::Vector3d::UnitX());
  Eigen::Quaterniond quat_desired(current);
  Eigen::Vector3d diff=quaternionsSubstraction(quat_desired, quat_current);
  Eigen::MatrixXd cartesian_vel(6,1);
  cartesian_vel(0,0)=goal_(0,0)-cartpos.p.x() + getCartesianOffset()(0,0);
  cartesian_vel(1,0)=goal_(1,0)-cartpos.p.y() + getCartesianOffset()(1,0);
  cartesian_vel(2,0)=goal_(2,0)-cartpos.p.z() + getCartesianOffset()(2,0);
  cartesian_vel(3,0)=diff[0];
  cartesian_vel(4,0)=diff[1];
  cartesian_vel(5,0)=diff[2];
  for(int i=0; i<cartesian_vel.rows(); i++){
    if(mask_cart_[i]==0){
      cartesian_vel(i,0)=0;
    }
  }
  cartesian_vel=limitCartesianVel(cartesian_vel);
  last_cartesian_vel_=cartesian_vel;
  return cartesian_vel;
}
task_priority::Error_msg GoalFixedPose::getMsg(Eigen::MatrixXd vels, std::vector<int> mask){
  task_priority::Error_msg msg;
  for(int i=0; i<last_cartesian_vel_.rows(); i++){
    if(mask[i]==0){
      msg.desired.push_back(0);
      msg.achived.push_back(0);
      msg.error.push_back(0);
    }
    else{
      msg.desired.push_back(last_cartesian_vel_(i,0));
      msg.achived.push_back(vels(i,0));
      msg.error.push_back(last_cartesian_vel_(i,0)-vels(i,0));
    }
  }
  return msg;
}


GoalROSPose::GoalROSPose(KDL::Chain chain, std::vector<int> mask_cart, std::string pose_topic, ros::NodeHandle &nh, std::vector<int> joints_relation, float max_cartesian_vel, float max_joint_vel):Goal(){
  KDL::Chain chain_odom;
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
  chain_odom.addChain(chain);

  fk_chain_=new KDL::ChainFkSolverPos_recursive(chain_odom);
  mask_cart_=mask_cart;
  joints_relation_=joints_relation;
  nh_=nh;
  setMaxCartesianVel(max_cartesian_vel);
  setMaxJointVel(max_joint_vel);

  pose_sub_=nh_.subscribe<geometry_msgs::Pose>(pose_topic, 1, &GoalROSPose::poseCallback, this);
}
GoalROSPose::~GoalROSPose(){}

void GoalROSPose::poseCallback(const geometry_msgs::Pose::ConstPtr &msg){
  std::cout<<"pose received"<<std::endl;
  goal_.orientation=msg->orientation;
  goal_.position=msg->position;
  setInitialized(true);
}

Eigen::MatrixXd GoalROSPose::getGoal(std::vector<float> joints, std::vector<float> odom){
  KDL::JntArray jq(odom.size()+joints_relation_.size());
  for(int i=0; i<odom.size(); i++){
    jq(i)=odom[i];
  }
  for(int i=0; i<joints_relation_.size(); i++){
    jq(i+odom.size())=joints[joints_relation_[i]];
  }

  KDL::Frame cartpos;
  fk_chain_->JntToCart(jq, cartpos);
  double x,y,z,w;
  cartpos.M.GetQuaternion(x,y,z,w);
  Eigen::Quaterniond quat_current(w,x,y,z);
  Eigen::Quaterniond quat_desired(goal_.orientation.w, goal_.orientation.x, goal_.orientation.y, goal_.orientation.z);
  Eigen::Vector3d diff=quaternionsSubstraction(quat_desired, quat_current);
  Eigen::MatrixXd cartesian_vel(6,1);
  cartesian_vel(0,0)=goal_.position.x-cartpos.p.x() +getCartesianOffset()(0,0);
  cartesian_vel(1,0)=goal_.position.y-cartpos.p.y() +getCartesianOffset()(1,0);
  cartesian_vel(2,0)=goal_.position.z-cartpos.p.z() +getCartesianOffset()(2,0);
  cartesian_vel(3,0)=diff[0];
  cartesian_vel(4,0)=diff[1];
  cartesian_vel(5,0)=diff[2];

  float eucl=0;
  for(int i=0; i<cartesian_vel.rows(); i++){
    eucl+=(cartesian_vel(i,0)*cartesian_vel(i,0));
  }
  eucl=std::sqrt(eucl);
  std::cout<<"euclidean error= "<<eucl<<std::endl;

  for(int i=0; i< 5; i++){
    if(cartesian_vel(i,0)>0.1){
      cartesian_vel(i,0)=cartesian_vel(i,0)*2;
    }
    else if(cartesian_vel(i,0)>0.05){
      cartesian_vel(i,0)=cartesian_vel(i,0)*1;
    }
    else if(cartesian_vel(i,0)>0.02){
      cartesian_vel(i,0)=cartesian_vel(i,0)*0.5;
    }
    else{
       cartesian_vel(i,0)=cartesian_vel(i,0)*0.3;
    }
  }

  for(int i=0; i<cartesian_vel.rows(); i++){
    if(mask_cart_[i]==0){
      cartesian_vel(i,0)=0;
    }
  }
  std::cout<<"cartesian_vel"<<std::endl;
  std::cout<<cartesian_vel<<std::endl;
  cartesian_vel=limitCartesianVel(cartesian_vel);
  last_cartesian_vel_=cartesian_vel;
  return cartesian_vel;
}

task_priority::Error_msg GoalROSPose::getMsg(Eigen::MatrixXd vels, std::vector<int> mask){
  task_priority::Error_msg msg;
  for(int i=0; i<last_cartesian_vel_.rows(); i++){
    if(mask[i]==0){
      msg.desired.push_back(0);
      msg.achived.push_back(0);
      msg.error.push_back(0);
    }
    else{
      msg.desired.push_back(last_cartesian_vel_(i,0));
      msg.achived.push_back(vels(i,0));
      msg.error.push_back(last_cartesian_vel_(i,0)-vels(i,0));
    }
  }
  return msg;

}

GoalROSTwist::GoalROSTwist(std::vector<int> mask_cart, std::string twist_topic, ros::NodeHandle &nh, float max_cartesian_vel, float max_joint_vel):Goal(){
  mask_cart_=mask_cart;
  nh_=nh;
  setMaxCartesianVel(max_cartesian_vel);
  setMaxJointVel(max_joint_vel);

  twist_sub_=nh_.subscribe<geometry_msgs::Twist>(twist_topic, 1, &GoalROSTwist::twistCallback, this);
}
GoalROSTwist::~GoalROSTwist(){}

void GoalROSTwist::twistCallback(const geometry_msgs::Twist::ConstPtr &msg){
  goal_.angular=msg->angular;
  goal_.linear=msg->linear;
  setInitialized(true);
}

Eigen::MatrixXd GoalROSTwist::getGoal(std::vector<float> joints, std::vector<float> odom){
  Eigen::MatrixXd cartesian_vel(6,1);
  cartesian_vel(0,0)=goal_.linear.x;
  cartesian_vel(1,0)=goal_.linear.y;
  cartesian_vel(2,0)=goal_.linear.z;
  cartesian_vel(3,0)=goal_.angular.x;
  cartesian_vel(4,0)=goal_.angular.y;
  cartesian_vel(5,0)=goal_.angular.z;
  cartesian_vel=limitCartesianVel(cartesian_vel);
  last_cartesian_vel_=cartesian_vel;
  return cartesian_vel;
}

task_priority::Error_msg GoalROSTwist::getMsg(Eigen::MatrixXd vels, std::vector<int> mask){
  task_priority::Error_msg msg;
  for(int i=0; i<last_cartesian_vel_.rows(); i++){
    if(mask[i]==0){
      msg.desired.push_back(0);
      msg.achived.push_back(0);
      msg.error.push_back(0);
    }
    else{
      msg.desired.push_back(last_cartesian_vel_(i,0));
      msg.achived.push_back(vels(i,0));
      msg.error.push_back(last_cartesian_vel_(i,0)-vels(i,0));
    }
  }
  return msg;
}

GoalJointsPosition::GoalJointsPosition(std::vector<float> joints_position, std::vector<int> mask_joint, float max_joint_vel):Goal(){
  joints_position_=joints_position;
  mask_joint_=mask_joint;
  setMaxJointVel(max_joint_vel);
  setInitialized(true);
}
GoalJointsPosition::~GoalJointsPosition(){}

Eigen::MatrixXd GoalJointsPosition::getGoal(std::vector<float> joints, std::vector<float> odom){
  Eigen::MatrixXd mat(joints_position_.size(),1);
  for(int i=0; i<joints_position_.size(); i++){
    if(mask_joint_[i]==0){
      mat(i,0)=0;
    }
    else{
      mat(i,0)=joints_position_[i]-joints[i];
    }
  }
  last_joint_vel_=mat;
  std::cout<<"mat antes"<<std::endl;
  std::cout<<mat<<std::endl;
  mat=limitJointVel(mat);
  std::cout<<"mat despues"<<std::endl;
  std::cout<<mat<<std::endl;
  return mat;
}

task_priority::Error_msg GoalJointsPosition::getMsg(Eigen::MatrixXd vels, std::vector<int> mask){
  task_priority::Error_msg msg;
  for(int i=0; i<last_joint_vel_.rows(); i++){
    if(mask[i]==0){
      msg.desired.push_back(0);
      msg.achived.push_back(0);
      msg.error.push_back(0);
    }
    else{
      msg.desired.push_back(last_joint_vel_(i,0));
      msg.achived.push_back(vels(i,0));
      msg.error.push_back(last_joint_vel_(i,0)-vels(i,0));
    }
  }
  return msg;
}





GoalGrasp::GoalGrasp(KDL::Chain chain, std::vector<int> mask_cart, std::string pose_topic, ros::NodeHandle &nh, std::vector<int> joints_relation, float max_cartesian_vel, float max_joint_vel, std::string joint_state_topic, std::string command_joint_topic, bool force_sensor, std::string force_sensor_topic, float max_force, std::string force_set_zero):Goal(){
  KDL::Chain chain_odom;
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
  chain_odom.addChain(chain);

  fk_chain_=new KDL::ChainFkSolverPos_recursive(chain_odom);
  mask_cart_=mask_cart;
  joints_relation_=joints_relation;
  nh_=nh;
  step_=0;
  /*grasp_client_= nh_.serviceClient<merbots_grasp_srv::grasp_srv>("/doGrasping");
  enable_keep_position_=nh_.serviceClient<std_srvs::Empty>("/cola2_control/enable_keep_position_4dof");
  disable_keep_position_=nh_.serviceClient<std_srvs::Empty>("/cola2_control/disable_keep_position");*/

  grasp_client_=nh_.serviceClient<merbots_grasp_srv::grasp_station_srv>("/doGrasping_Station");

  pose_received_=false;
  pose_reached_=0;

  joint_state_topic_=joint_state_topic;
  command_joint_topic_=command_joint_topic;
  force_detected_=false;
  force_direction_=0;


  setMaxCartesianVel(max_cartesian_vel);
  setMaxJointVel(max_joint_vel);

  pose_sub_=nh_.subscribe<geometry_msgs::Pose>(pose_topic, 1, &GoalGrasp::poseCallback, this);

  if(force_sensor){
    force_sub_=nh_.subscribe<geometry_msgs::WrenchStamped>(force_sensor_topic, 1, &GoalGrasp::forceCallback, this);
    max_force_=max_force;
    current_force_=0;
    force_set_zero_=nh_.serviceClient<std_srvs::Empty>(force_set_zero);
    // Forzar inizialización.
    reset_offset_next_callback_ = true;
    last_detection_=false;
    force_detected_=false;
  }

}

GoalGrasp::~GoalGrasp(){}

void GoalGrasp::forceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  std::cout<<msg->wrench.torque.y<<"    "<<msg->wrench.torque.z<<"    "<<force_direction_<<std::endl;

  if( reset_offset_next_callback_ ){
    force_offfset_torque_y_ = msg->wrench.torque.y;
    force_offfset_torque_z_ = msg->wrench.torque.z;
    reset_offset_next_callback_ = false;
  }

  double torque_y, torque_z;
  torque_y = msg->wrench.torque.y - force_offfset_torque_y_;
  torque_z = msg->wrench.torque.z - force_offfset_torque_z_;

  std::cout<<"Corrected y:"<<torque_y<<"  z  "<<torque_z<<std::endl;

  if(std::abs(torque_y)> max_force_ || std::abs(torque_z)> max_force_){
    if(std::abs(torque_y)>std::abs(torque_z)){
      if(torque_y>max_force_){
        force_detected_=true;
        current_force_=torque_y;
        if(force_direction_==0){
          force_direction_=1;
        }
      }
      else if(torque_y<-max_force_){
        force_detected_=true;
        current_force_=torque_y;
        if(force_direction_==0){
          force_direction_=-1;
        }
      }
    }
    else{
      if(torque_z>max_force_){
        force_detected_=true;
        current_force_=torque_z;
        if(force_direction_==0){
          force_direction_=-1;
        }
      }
      else if(torque_z<-max_force_){
        force_detected_=true;
        current_force_=torque_z;
        if(force_direction_==0){
          force_direction_=1;
        }
      }
    }
    ROS_INFO("Force detected: %f", current_force_);
  }
  else{
    force_detected_=false;
    last_detection_=false;
    force_direction_=0;
    current_force_=0;
  }

}

void GoalGrasp::poseCallback(const geometry_msgs::Pose::ConstPtr &msg){
  //if(!pose_received_){
    //std::cout<<"pose received"<<std::endl;
    goal_.orientation=msg->orientation;
    goal_.position=msg->position;
    setInitialized(true);
    pose_received_=true;
  //}
}

Eigen::MatrixXd GoalGrasp::getGoal(std::vector<float> joints, std::vector<float> odom){
  KDL::JntArray jq(odom.size()+joints_relation_.size());
  for(int i=0; i<odom.size(); i++){
    jq(i)=odom[i];
  }
  for(int i=0; i<joints_relation_.size(); i++){
    jq(i+odom.size())=joints[joints_relation_[i]];
  }

  KDL::Frame cartpos;
  fk_chain_->JntToCart(jq, cartpos);
  double x,y,z,w;
  cartpos.M.GetQuaternion(x,y,z,w);
  Eigen::Quaterniond quat_current(w,x,y,z);
  Eigen::Quaterniond quat_desired(goal_.orientation.w, goal_.orientation.x, goal_.orientation.y, goal_.orientation.z);
  Eigen::Vector3d diff=quaternionsSubstraction(quat_desired, quat_current);

  /*///// Force Sensor handle
  if(force_detected_ && ! last_detection_){
    last_detection_=true;
    Eigen::MatrixXd increment(3,1);
    increment.setZero();
    if(((goal_.position.z)-cartpos.p.z() +getCartesianOffset()(2,0))>0.05){

      if(force_direction_==1){
        Eigen::Transform<double, 3, Eigen::Affine> wMdir=Eigen::Translation3d(cartpos.p.x(), cartpos.p.y(),cartpos.p.z())*quat_current*Eigen::Translation3d(0.05, 0.0, 0.0);
        increment(0,0)=wMdir(0,3)-cartpos.p.x();
        increment(1,0)=wMdir(1,3)-cartpos.p.y();
        incrementOffset(increment);
      }
      else if(force_direction_==-1){
        Eigen::Transform<double, 3, Eigen::Affine> wMdir=Eigen::Translation3d(cartpos.p.x(), cartpos.p.y(),cartpos.p.z())*quat_current*Eigen::Translation3d(-0.05, 0.0, 0.0);
        increment(0,0)=wMdir(0,3)-cartpos.p.x();
        increment(1,0)=wMdir(1,3)-cartpos.p.y();
        incrementOffset(increment);
      }
    }
    else{
      Eigen::Transform<double, 3, Eigen::Affine> wMdir=Eigen::Translation3d(cartpos.p.x(), cartpos.p.y(),cartpos.p.z())*quat_current*Eigen::Translation3d(0.0, 0.0, -0.03);
      increment(0,0)=wMdir(0,3)-cartpos.p.x();
      increment(1,0)=wMdir(1,3)-cartpos.p.y();
      incrementOffset(increment);
    }
    last_detection_=true;
  }

  /////*/


  Eigen::MatrixXd cartesian_vel(6,1);
  cartesian_vel(0,0)=goal_.position.x-cartpos.p.x()  +getCartesianOffset()(0,0);
  cartesian_vel(1,0)=goal_.position.y-cartpos.p.y()  +getCartesianOffset()(1,0);
  if(force_detected_){
    cartesian_vel(2,0)=-0.3;
    step_=0;
  }
  else{
    if(step_==0){
      cartesian_vel(2,0)=(goal_.position.z-0.3)-cartpos.p.z()  +getCartesianOffset()(2,0);
    }
    else if(step_==1){
      cartesian_vel(2,0)=(goal_.position.z)-cartpos.p.z() +getCartesianOffset()(2,0);
    }
    else if(step_==3){
      cartesian_vel(2,0)=-4.0;
    }
  }
  cartesian_vel(3,0)=diff[0];
  cartesian_vel(4,0)=diff[1];
  cartesian_vel(5,0)=diff[2];
  for(int i=0; i<cartesian_vel.rows(); i++){
    if(mask_cart_[i]==0){
      cartesian_vel(i,0)=0;
    }
  }


  float eucl=0, eucl_deg;
  for(int i=0; i<3; i++){
    eucl+=(cartesian_vel(i,0)*cartesian_vel(i,0));
    eucl_deg+=(cartesian_vel(i+3,0)*cartesian_vel(i+3,0));
  }
  std::cout<<"cartesian_vel"<<std::endl;
  std::cout<<cartesian_vel<<std::endl;
  eucl=std::sqrt(eucl);
  eucl_deg=std::sqrt(eucl_deg);
  std::cout<<"euclidean error= "<<eucl<<std::endl;
  std::cout<<"euclidean degree error= "<<eucl_deg<<std::endl;

  for(int i=0; i< 5; i++){
    if(step_==1){
      cartesian_vel(i,0)=cartesian_vel(i,0)*0.3;
    }
    else{
      if(cartesian_vel(i,0)>0.1){
        cartesian_vel(i,0)=cartesian_vel(i,0)*2;
      }
      else if(cartesian_vel(i,0)>0.05){
        cartesian_vel(i,0)=cartesian_vel(i,0)*1;
      }
      else if(cartesian_vel(i,0)>0.02){
        cartesian_vel(i,0)=cartesian_vel(i,0)*0.5;
      }
      else{
        cartesian_vel(i,0)=cartesian_vel(i,0)*0.3;
      }
    }
  }



  std::cout<<"desired   ------    current"<<std::endl;
  std::cout<<goal_.position.x<<"   ---  "<<cartpos.p.x()<<std::endl;
  std::cout<<goal_.position.y<<"   ---  "<<cartpos.p.y()<<std::endl;
  std::cout<<goal_.position.z<<"   ---  "<<cartpos.p.z()<<std::endl;
  std::cout<<quat_desired.x()<<"   ---  "<<quat_current.x()<<std::endl;
  std::cout<<quat_desired.y()<<"   ---  "<<quat_current.y()<<std::endl;
  std::cout<<quat_desired.z()<<"   ---  "<<quat_current.z()<<std::endl;
  std::cout<<quat_desired.w()<<"   ---  "<<quat_current.w()<<std::endl;



  if(eucl<0.03 && eucl_deg<0.15){
    pose_reached_++;
    if(pose_reached_>=5){
      step_++;
      std_srvs::Empty empty;
      force_set_zero_.call(empty);
      // Forzar inizialización.
      reset_offset_next_callback_ = true;
      pose_reached_=0;
      if(step_==2){
        /*std_srvs::Empty empty_srv;
        if(enable_keep_position_.call(empty_srv)){

          merbots_grasp_srv::grasp_srv srv;
          srv.request.command_topic="/arm5e/command_angle";
          srv.request.joint_state_topic="/arm5e/joint_state_angle";
          srv.request.grasped_current=1.4;
          if(grasp_client_.call(srv)){
            if(srv.response.success){
              step_++;
            }
            else{
              step_=0;
            }
          }
          else{
            ROS_ERROR("Failed to call service doGrasping");
          }
          while(!disable_keep_position_.call(empty_srv)){
            usleep(10000);
            ros::spinOnce();
          }
        }*/

        merbots_grasp_srv::grasp_station_srv srv;
        srv.request.command_topic=command_joint_topic_;
        srv.request.joint_state_topic=joint_state_topic_;
        srv.request.grasped_current=1.6;
        srv.request.station_topic="/cola2_control/world_waypoint_req";
        srv.request.north=odom[0];
        srv.request.east=odom[1];
        srv.request.depth=odom[2];
        srv.request.roll=odom[3];
        srv.request.pitch=odom[4];
        srv.request.yaw=odom[5];
        if(grasp_client_.call(srv)){
          if(srv.response.success){
            step_++;
          }
          else{
            step_=0;
          }
        }
        else{
          ROS_ERROR("Failed to call service doGrasping_Station");
        }
      }
    }
  }
  else{
    pose_reached_=0;
  }
  cartesian_vel=cartesian_vel*0.5;
  cartesian_vel=limitCartesianVel(cartesian_vel);
 std::cout<<"Step="<<step_<<std::endl;
  std::cout<<"Cartesian_vel"<<std::endl;
  std::cout<<cartesian_vel<<std::endl;
  last_cartesian_vel_=cartesian_vel;
return cartesian_vel;
}

task_priority::Error_msg GoalGrasp::getMsg(Eigen::MatrixXd vels, std::vector<int> mask){
  task_priority::Error_msg msg;
  for(int i=0; i<last_cartesian_vel_.rows(); i++){
    if(mask[i]==0){
      msg.desired.push_back(0);
      msg.achived.push_back(0);
      msg.error.push_back(0);
    }
    else{
      msg.desired.push_back(last_cartesian_vel_(i,0));
      msg.achived.push_back(vels(i,0));
      msg.error.push_back(last_cartesian_vel_(i,0)-vels(i,0));
    }
  }
  return msg;

}






//// Goal Joint Position
void GoalROSJointsState::jointsCallback(const sensor_msgs::JointState::ConstPtr &msg){
  joints_position_.resize(9);
  for(int i=0; i<4; i++){
    joints_position_[i]=msg->position[i];
  }
  joints_position_[4]=0.0;
  for(int i=5; i<9; i++){
    joints_position_[i]=msg->position[i-1];
  }
  setInitialized(true);
}

void GoalROSJointsState::graspingCallback(const std_msgs::String::ConstPtr &msg){
  if(msg->data=="approach"){
    grasping_=0;
  }
  if(msg->data=="grasp"){
    grasping_=1;
  }
  if(msg->data=="done"){
    grasping_=2;
  }
}

GoalROSJointsState::GoalROSJointsState(std::string topic_name, ros::NodeHandle nh):Goal(){
  nh_=nh;
  joint_state_sub_=nh_.subscribe<sensor_msgs::JointState>(topic_name, 1, &GoalROSJointsState::jointsCallback, this);
  finished_pub_=nh_.advertise<std_msgs::Bool>("/plan_status", 1);
  grasping_=0;
  grasping_sub_=nh_.subscribe<std_msgs::String>("/grasp_status", 1, &GoalROSJointsState::graspingCallback, this);
  grasp_client_=nh_.serviceClient<merbots_grasp_srv::grasp_station_srv>("/doGrasping_Station");
  setInitialized(false);
}
GoalROSJointsState::~GoalROSJointsState(){}

Eigen::MatrixXd GoalROSJointsState::getGoal(std::vector<float> joints, std::vector<float> odom){

  KDL::Chain auv;
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));

  KDL::ChainJntToJacSolver jntToJac(auv);
  KDL::JntArray q(auv.getNrOfJoints());
  for(int i=0; i<6; i++){
    q(i)=odom[i];
  }
  for(int i=6; i<10; i++){
    q(i)=0;
  }

  KDL::Jacobian jac(auv.getNrOfJoints());
  jntToJac.JntToJac(q, jac);
  Eigen::MatrixXd jacobian(jac.rows(), 4);
  jacobian.setZero();

  for(int j=0; j<4; j++){
    for(int i=0; i<jac.rows(); i++){
      jacobian(i,j)=jac(i,j+6);
    }
  }
  std::cout<<jacobian<<std::endl;
  Eigen::MatrixXd inv_jac=pinvMat(jacobian);

  Eigen::MatrixXd values(6,1);
  values(0,0)=joints_position_[0]-odom[0];
  values(1,0)=joints_position_[1]-odom[1];
  values(2,0)=joints_position_[2]-odom[2];
  values(3,0)=0;
  values(4,0)=0;

  values(5,0)=(joints_position_[3]-odom[5])+2*M_PI*std::floor((M_PI-(joints_position_[3]-odom[5]))/(2*M_PI));

  Eigen::MatrixXd vels=inv_jac*values;
  std::cout<<std::endl<<vels<<std::endl;

  Eigen::MatrixXd mat(9,1);
  mat(0,0)=vels(0,0);
  mat(1,0)=vels(1,0);
  mat(2,0)=vels(2,0);
  mat(3,0)=vels(3,0);

  mat(4,0)=0;

  for(int i=5; i<9; i++){
    mat(i,0)=joints_position_[i]-joints[i];
  }


  if(grasping_==0){
    bool finished=true;
    for(int i=0; i<3; i++){
      if(mat(i,0)>0.05)
        finished=false;
    }
    for(int i=4; i<9; i++){
      if(mat(i,0)>0.1)
        finished=false;
    }
    if(finished){
      std_msgs::Bool bool_msg;
      bool_msg.data=true;
      finished_pub_.publish(bool_msg);
      finished=false;
    }

    for(int i=0; i<4; i++){
      mat(i,0)=mat(i,0)*0.4;
    }
    for(int i=4; i<9; i++){
      mat(i,0)=mat(i,0)*2;
    }
  }


  if(grasping_==1){
    bool finished=true;
    for(int i=0; i<3; i++){
      if(mat(i,0)>0.01)
        finished=false;
    }
    for(int i=4; i<9; i++){
      if(mat(i,0)>0.05)
        finished=false;
    }
    if(finished){
      std_msgs::Bool bool_msg;
      bool_msg.data=true;
      finished_pub_.publish(bool_msg);
      finished=false;
    }
    for(int i=0; i<4; i++){
      if(mat(i,0)>0.1){
        mat(i,0)=mat(i,0)*2;
      }
      else if(mat(i,0)>0.05){
        mat(i,0)=mat(i,0)*1;
      }
      else if(mat(i,0)>0.02){
        mat(i,0)=mat(i,0)*0.5;
      }
      else{
        mat(i,0)=mat(i,0)*0.3;
      }
    }
    for(int i=4; i<9; i++){
      mat(i,0)=mat(i,0)*2;
    }
  }
  if(grasping_==2){
    std::cout<<"into sleep function"<<std::endl;
    /*merbots_grasp_srv::grasp_station_srv srv;
    srv.request.command_topic="/arm5e/command_angle";
    srv.request.joint_state_topic="/arm5e/joint_state_angle_fixed";
    srv.request.grasped_current=1.6;
    srv.request.station_topic="/cola2_control/world_waypoint_req";
    srv.request.north=odom[0];
    srv.request.east=odom[1];
    srv.request.depth=odom[2];
    srv.request.roll=odom[3];
    srv.request.pitch=odom[4];
    srv.request.yaw=odom[5];
    grasp_client_.call(srv);*/
    sleep(10000);
  }



  std::cout<<std::endl<<std::endl;
  for(int i=0; i<3; i++){
    std::cout<<joints_position_[i]<<"  --  "<<odom[i]<<"  --  "<<mat(i,0)<<std::endl;
  }
  std::cout<<joints_position_[3]<<"  --  "<<odom[5]<<"  --  "<<mat(3,0)<<std::endl;

  for(int i=5; i<9; i++){
    std::cout<<joints_position_[i]<<"  --  "<<joints[i]<<"  --  "<<mat(i,0)<<std::endl;
  }


  last_joint_vel_=mat;
  return mat;
}

task_priority::Error_msg GoalROSJointsState::getMsg(Eigen::MatrixXd vels, std::vector<int> mask){
  task_priority::Error_msg msg;
  for(int i=0; i<last_joint_vel_.rows(); i++){
    if(mask[i]==0){
      msg.desired.push_back(0);
      msg.achived.push_back(0);
      msg.error.push_back(0);
    }
    else{
      msg.desired.push_back(last_joint_vel_(i,0));
      msg.achived.push_back(vels(i,0));
      msg.error.push_back(last_joint_vel_(i,0)-vels(i,0));
    }
  }
  return msg;
}
