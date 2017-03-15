#include <task_priority/goal.hpp>


Goal::Goal(){
  initialized_=false;
  cartesian_offset_.resize(3,1);
  cartesian_offset_.setZero();
}
Goal::~Goal(){}

Eigen::Vector3d Goal::quaternionsSubstraction(Eigen::Quaterniond quat_desired, Eigen::Quaterniond quat_current){
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
}
void Goal::setMaxCartesianVel(float max_cartesian_vel){
  max_cartesian_vel_=max_cartesian_vel;
}

Eigen::MatrixXd Goal::limitCaresianVel(Eigen::MatrixXd vels){
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



GoalFixedPose::GoalFixedPose(Eigen::MatrixXd goal, KDL::Chain chain, std::vector<int> mask_cart, std::vector<int> joints_relation, float max_cartesian_vel):Goal(){
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
  cartesian_vel=limitCaresianVel(cartesian_vel);
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


GoalROSPose::GoalROSPose(KDL::Chain chain, std::vector<int> mask_cart, std::string pose_topic, ros::NodeHandle &nh, std::vector<int> joints_relation, float max_cartesian_vel):Goal(){
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
  for(int i=0; i<cartesian_vel.rows(); i++){
    if(mask_cart_[i]==0){
      cartesian_vel(i,0)=0;
    }
  }
  cartesian_vel=limitCaresianVel(cartesian_vel);
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

GoalROSTwist::GoalROSTwist(std::vector<int> mask_cart, std::string twist_topic, ros::NodeHandle &nh, float max_cartesian_vel):Goal(){
  mask_cart_=mask_cart;
  nh_=nh;
  setMaxCartesianVel(max_cartesian_vel);

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
  cartesian_vel=limitCaresianVel(cartesian_vel);
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

GoalJointsPosition::GoalJointsPosition(std::vector<float> joints_position):Goal(){
  joints_position_=joints_position;
  setInitialized(true);
}
GoalJointsPosition::~GoalJointsPosition(){}

Eigen::MatrixXd GoalJointsPosition::getGoal(std::vector<float> joints, std::vector<float> odom){
  Eigen::MatrixXd mat(joints_position_.size(),1);
  for(int i=0; i<joints_position_.size(); i++){
    mat(i,0)=joints_position_[i]-joints[i];
  }
  last_joint_vel_=mat;
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





GoalGrasp::GoalGrasp(KDL::Chain chain, std::vector<int> mask_cart, std::string pose_topic, ros::NodeHandle &nh, std::vector<int> joints_relation, float max_cartesian_vel):Goal(){
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
  grasp_client_= nh_.serviceClient<merbots_grasp_srv::grasp_srv>("/doGrasping");
  enable_keep_position_=nh_.serviceClient<std_srvs::Empty>("/cola2_control/enable_keep_position_4dof");
  disable_keep_position_=nh_.serviceClient<std_srvs::Empty>("/cola2_control/disable_keep_position");
  pose_received_=false;
  pose_reached_=0;


  setMaxCartesianVel(max_cartesian_vel);

  pose_sub_=nh_.subscribe<geometry_msgs::Pose>(pose_topic, 1, &GoalGrasp::poseCallback, this);
}
GoalGrasp::~GoalGrasp(){}

void GoalGrasp::poseCallback(const geometry_msgs::Pose::ConstPtr &msg){
  if(!pose_received_){
    std::cout<<"pose received"<<std::endl;
    goal_.orientation=msg->orientation;
    goal_.position=msg->position;
    setInitialized(true);
    pose_received_=true;
  }
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
  Eigen::MatrixXd cartesian_vel(6,1);
  cartesian_vel(0,0)=goal_.position.x-cartpos.p.x()  +getCartesianOffset()(0,0);
  cartesian_vel(1,0)=goal_.position.y-cartpos.p.y()  +getCartesianOffset()(1,0);
  if(step_==0){
    cartesian_vel(2,0)=(goal_.position.z-0.2)-cartpos.p.z()  +getCartesianOffset()(2,0);
  }
  else if(step_==1){
    cartesian_vel(2,0)=(goal_.position.z)-cartpos.p.z() +getCartesianOffset()(2,0);
  }
  else if(step_==3){
    cartesian_vel(2,0)=-1.0;
  }
  cartesian_vel(3,0)=diff[0];
  cartesian_vel(4,0)=diff[1];
  cartesian_vel(5,0)=diff[2];
  for(int i=0; i<cartesian_vel.rows(); i++){
    if(mask_cart_[i]==0){
      cartesian_vel(i,0)=0;
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

  cartesian_vel=limitCaresianVel(cartesian_vel);
  std::cout<<"Step="<<step_<<std::endl;
  std::cout<<"Cartesian_vel"<<std::endl;
  std::cout<<cartesian_vel<<std::endl;
  last_cartesian_vel_=cartesian_vel;
  float eucl=0;
  for(int i=0; i<cartesian_vel.rows(); i++){
    eucl+=(cartesian_vel(i,0)*cartesian_vel(i,0));
  }
  eucl=std::sqrt(eucl);
  std::cout<<"euclidean error= "<<eucl<<std::endl;
  if(eucl<0.03){
    pose_reached_++;
    if(pose_reached_>=5){
      step_++;
      pose_reached_=0;
      if(step_==2){
        std_srvs::Empty empty_srv;
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
        }

      }
    }
  }
  else{
    pose_reached_=0;
  }
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
