#include <task_priority/grasp_planning_controller.hpp>
#include <task_priority/TaskPriority_msg.h>
#include <task_priority/HardConstraints_msg.h>



GraspController::GraspController(std::vector<MultiTaskPtr> multitasks, int n_joints, std::vector<float> max_joint_limit, std::vector<float> min_joint_limit, std::vector<std::vector<float> > max_cartesian_limits, std::vector<std::vector<float> > min_cartesian_limits, float max_cartesian_vel, float acceleration, float max_joint_vel, float sampling_duration, ros::NodeHandle nh, std::string arm_joint_state_topic, std::string arm_joint_command_topic, std::string vehicle_tf, std::string world_tf, std::string vehicle_command_topic, std::vector<KDL::Chain> chains, std::vector<std::vector<int> > chain_joint_relations, bool simulation, std::vector<float> p_values, std::vector<float> i_values, std::vector<float> d_values){
multitasks_=multitasks;
n_joints_=n_joints;
max_joint_limit_=max_joint_limit;
min_joint_limit_=min_joint_limit;
std::cout<<"Max Joint limit"<<std::endl;
max_cartesian_limits_=max_cartesian_limits;
min_cartesian_limits_=min_cartesian_limits;
max_cartesian_vel_=max_cartesian_vel;
acceleration_=acceleration;
max_joint_vel_=max_joint_vel;
sampling_duration_=sampling_duration;
world_tf_=world_tf;
vehicle_tf_=vehicle_tf;
nh_=nh;
joints_init_=false;
goal_init_=false;
current_joints_.resize(n_joints);
chains_=chains;
chain_joint_relations_=chain_joint_relations;
simulation_=simulation;



///////////////////PI Controller/////////////////////

//pi_controller_.reset(new PIController(p_values, i_values, d_values));


//////////////////////////////////77




//joints_sub_=nh_.subscribe<sensor_msgs::JointState>(arm_joint_state_topic, 1, &GraspController::jointsCallback, this);
joints_pub_=nh_.advertise<sensor_msgs::JointState>(arm_joint_command_topic,1);

if(simulation){
  vehicle_pub_=nh_.advertise<nav_msgs::Odometry>(vehicle_command_topic, 1);

}
else
  vehicle_pub_=nh_.advertise<auv_msgs::BodyVelocityReq>(vehicle_command_topic, 1);

effector_twist_=nh_.advertise<geometry_msgs::TwistStamped>("/task_priority/effector_twist", 1);
vehicle_twist_=nh_.advertise<geometry_msgs::TwistStamped>("/task_priority/vehicle_twist", 1);
effector_pose_=nh_.advertise<geometry_msgs::PoseStamped>("/task_priority/effector_pose", 1);
vehicle_pose_=nh_.advertise<geometry_msgs::PoseStamped>("/task_priority/vehicle_pose", 1);
status_pub_=nh_.advertise<task_priority::TaskPriority_msg>("/task_priority/status", 1);
constraints_pub_=nh_.advertise<task_priority::HardConstraints_msg>("/task_priority/hard_constraints", 1);
}

GraspController::~GraspController(){}




/*void GraspController::jointsCallback(const sensor_msgs::JointStateConstPtr &msg){
  if(simulation_){
    for(int i=0; i<4; i++){
      current_joints_[i]=0.0;
    }
    for(int i=4; i<8; i++){
      current_joints_[i]=msg->position[i-4];
    }
  }
  else{
    for(int i=0; i<5; i++){
      current_joints_[i]=0.0;
    }

    for(int i=5; i<9; i++){
      current_joints_[i]=msg->position[i-5];
    }
  }
  joints_init_=true;
}*/

float GraspController::calculateMaxPositiveVel(float current_joint, float max_joint_value, float acceleration, float sampling_duration){
  if(current_joint>max_joint_value)
    return 0.0;
  return std::max(std::min((max_joint_value-current_joint)/sampling_duration/5, std::sqrt(2*acceleration*(max_joint_value-current_joint))),(float)0.0);
}

float GraspController::calculateMaxNegativeVel(float current_joint, float min_joint_value, float acceleration, float sampling_duration){
  if(current_joint<min_joint_value)
    return 0.0;
  return std::min(std::max((min_joint_value-current_joint)/sampling_duration/5, -std::sqrt(2*acceleration*(current_joint-min_joint_value))),(float)0.0);
}

float GraspController::calculateMaxPositiveVel(float difference, float acceleration, float sampling_duration){
  if(difference<0)
    return 0.0;
  return std::max(std::min(difference/sampling_duration/5, std::sqrt(2*acceleration*difference)),(float)0.0);
}

float GraspController::calculateMaxNegativeVel(float difference, float acceleration, float sampling_duration){
  if(difference>0)
    return 0.0;
  return std::min(std::max(difference/sampling_duration/5, -std::sqrt(2*acceleration*(-difference))),(float)0.0);
}

Eigen::Vector3d GraspController::quaternionsSubstraction(Eigen::Quaterniond quat_desired, Eigen::Quaterniond quat_current){
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
}

std::vector<float> GraspController::getSolution(std::vector<float> joints, std::vector<float> odom, std::vector<int> important_tasks){
  bool initialized=false;
  current_joints_=joints;




  while(ros::ok() && !initialized){

    initialized=true;
    for(int i=0; i<multitasks_.size(); i++){
      if(!multitasks_[i]->goalsInitialized()){
        initialized=false;
        std::cout<<"Task "<<i<<" not initialized"<<std::endl;
      }
    }


    ros::Duration(sampling_duration_).sleep();
    ros::spinOnce();
  }
  std::cout<<"Topics initialized"<<std::endl;
  bool exit=false;


  do{

    /*try{
      tf::StampedTransform transform;
      listener.lookupTransform(world_tf_, vehicle_tf_, ros::Time(0), transform);
      Eigen::Quaterniond odom_rotation(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
      Eigen::Vector3d odom_euler=odom_rotation.toRotationMatrix().eulerAngles(2,1,0);
      std::vector<float> odom(6,0);
      odom[0]=transform.getOrigin().x();
      odom[1]=transform.getOrigin().y();
      odom[2]=transform.getOrigin().z();
      odom[3]=odom_euler[2];
      odom[4]=odom_euler[1];
      odom[5]=odom_euler[0];*/






    std::vector<float> max_positive_joint_velocities, max_negative_joint_velocities;
    for(int i=0; i<n_joints_; i++){
      max_positive_joint_velocities.push_back(calculateMaxPositiveVel(current_joints_[i], max_joint_limit_[i], acceleration_, sampling_duration_));
      //std::cout<< i<<"     "<<current_joints_[i]<<"    "<<max_joint_limit_[i]<<"    "<<max_positive_joint_velocities[i]<<std::endl;
      max_negative_joint_velocities.push_back(calculateMaxNegativeVel(current_joints_[i], min_joint_limit_[i], acceleration_, sampling_duration_));
      //std::cout<<max_positive_joint_velocities[i]<<"-----    "<<max_negative_joint_velocities[i]<<std::endl;
    }
    std::vector<std::vector<float> > max_negative_cartesian_velocities, max_positive_cartesian_velocities;
    std::vector<std::vector<std::vector<float> > > max_cartesian_vels=calculateMaxCartesianVels(current_joints_, odom);
    max_positive_cartesian_velocities=max_cartesian_vels[0];
    max_negative_cartesian_velocities=max_cartesian_vels[1];
    Eigen::MatrixXd T_k_complete;
    Eigen::MatrixXd vels(n_joints_,1);
    vels.setZero();

    for(int i=multitasks_.size()-1; i>=0; i--){
      //std::cout<<"Multitask "<<i<<std::endl;
      multitasks_[i]->setCurrentJoints(current_joints_);
      multitasks_[i]->setOdom(odom);


      multitasks_[i]->setMaxPositiveJointVelocity(max_positive_joint_velocities);
      multitasks_[i]->setMaxNegativeJointVelocity(max_negative_joint_velocities);
      multitasks_[i]->setMaxPositiveCartesianVelocity(max_positive_cartesian_velocities);
      multitasks_[i]->setMaxNegativeCartesianVelocity(max_negative_cartesian_velocities);

      vels=multitasks_[i]->calculateMultiTaskVel(vels, T_k_complete);
      //std::cout<<"Vels at the end of multitatsk "<<i<<std::endl;
      //std::cout<<vels<<std::endl;
      T_k_complete=multitasks_[i]->getT_k_complete();
    }

    vels=limitVels(vels);
    /*std::cout<<"vels to publish "<<std::endl;
    std::cout<<vels<<std::endl;
    std::cout<<"----------------------------"<<std::endl;*/




    Eigen::MatrixXd vels_zero(n_joints_,1);
    vels_zero.setZero();
    float error_total=0;
    for(int j=0; j<important_tasks.size(); j++){
      Eigen::MatrixXd error_zero=multitasks_[important_tasks[j]]->calculateError(vels_zero);
      Eigen::MatrixXd error=multitasks_[important_tasks[j]]->calculateError(vels);
      for(int i=0; i<error.rows(); i++){
        std::cout<<error(i,0)<<" --- "<<error_zero(i,0)<<std::endl;
        error_total+=std::abs(error(i,0))-std::abs(error_zero(i,0));
      }
    }
    std::cout<<"Error= "<<error_total<<std::endl;
    if(error_total>0 && isInsideLimits(odom)){
      std::cout<<"Salgo por error positivo"<<std::endl;

      return joints;
    }

    float vels_sum=0;
    for(int i=4; i<8; i++){
      current_joints_[i]+=0.5*vels(i,0);
      vels_sum+=std::abs(vels(i,0));
    }
    vels_sum/=4;
    std::cout<<"vels_sum= "<<vels_sum<<std::endl;
    if(vels_sum<0.001 && isInsideLimits(odom)){
      exit=true;
      sleep(10);
      return current_joints_;
    }

    std::cout<<isInsideLimits(odom)<<std::endl;
    //publishVels(vels);
    publishJoints(current_joints_);


    publishTwist(vels, odom);
    publishPose(odom);
    publishConstraints();




    /*}
    catch(tf::TransformException ex){
      ROS_ERROR("%s\n", ex.what());
      ros::Duration(sampling_duration_/10).sleep();
    }*/
    //ros::Duration(sampling_duration_).sleep();
  }while(ros::ok() && !exit);

  return joints;
}


Eigen::MatrixXd GraspController::limitVels(Eigen::MatrixXd vels){
  float max_vel=0;
  for(int i=0; i<vels.rows(); i++){
    if(std::abs(vels(i,0))>max_vel){
      max_vel=std::abs(vels(i,0));
    }
  }
  if(max_vel>max_joint_vel_){
    for(int i=0; i<vels.rows(); i++){
      vels(i,0)=vels(i,0)*max_joint_vel_/max_vel;
    }
  }
  return vels;
}

void GraspController::publishVels(Eigen::MatrixXd vels){



  sensor_msgs::JointState joint_msg;
  joint_msg.name.push_back("Slew");
  joint_msg.name.push_back("Shoulder");
  joint_msg.name.push_back("Elbow");
  joint_msg.name.push_back("JawRotate");
  joint_msg.name.push_back("JawOpening");

  if(simulation_){
    for(int i=4; i<8; i++){
      joint_msg.velocity.push_back(vels(i,0));
    }
  }
  else{
    for(int i=5; i<9; i++){
      joint_msg.velocity.push_back(5*vels(i,0));
    }
  }
  joint_msg.velocity.push_back(0);
  joint_msg.header.stamp=ros::Time::now();

  /*if(simulation_)
    vehicle_pub_.publish(odom_msg);
  else
    vehicle_pub_.publish(vehicle_msg);*/
  joints_pub_.publish(joint_msg);
  publishStatus(vels);

  ros::spinOnce();
}

void GraspController::publishJoints(std::vector<float> joints){
  sensor_msgs::JointState joint_msg;
  joint_msg.name.push_back("Slew");
  joint_msg.name.push_back("Shoulder");
  joint_msg.name.push_back("Elbow");
  joint_msg.name.push_back("JawRotate");
  joint_msg.name.push_back("JawOpening");
  for(int i=4; i<8; i++){
    joint_msg.position.push_back(joints[i]);
  }
  joint_msg.velocity.push_back(0);
  joint_msg.header.stamp=ros::Time::now();
  joints_pub_.publish(joint_msg);

}

std::vector<std::vector<std::vector<float> > > GraspController::calculateMaxCartesianVels(std::vector<float> joints, std::vector<float> odom){
  std::vector<std::vector<float> > max_negative_cartesian_vels, max_positive_cartesian_vels;
  for(int i=0; i<chains_.size(); i++){
    KDL::Chain chain_odom;
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    chain_odom.addChain(chains_[i]);
    KDL::ChainFkSolverPos_recursive fk(chain_odom);

    KDL::JntArray q(chain_odom.getNrOfJoints());
    for(int j=0; j<odom.size(); j++){
      q(j)=odom[j];

    }
    for(int j=0; j<chains_[i].getNrOfJoints(); j++){
      q(j+odom.size())=joints[chain_joint_relations_[i][j]];
    }
    //for(int j=0; j<q.rows(); j++){
     // std::cout<<q(j)<<"  ";
    //}
    //std::cout<<std::endl;

    KDL::Frame frame;
    fk.JntToCart(q, frame);
    std::vector<float> max_negative_cartesian_vel, max_positive_cartesian_vel;
    //std::cout<<"Direct kinematics"<<std::endl;

    //std::cout<<frame.p.x()<<" "<<frame.p.y()<<" "<<frame.p.z()<<std::endl;
    double x_a, y_a, z_a, w_a;
    frame.M.GetQuaternion(x_a, y_a, z_a, w_a);
    //std::cout<<x_a<<" "<<y_a<<" "<<z_a<<" "<<w_a<<std::endl;

    max_negative_cartesian_vel.push_back(std::max(calculateMaxNegativeVel(frame.p.x(), min_cartesian_limits_[i][0], acceleration_, sampling_duration_), -max_cartesian_vel_));
    max_negative_cartesian_vel.push_back(std::max(calculateMaxNegativeVel(frame.p.y(), min_cartesian_limits_[i][1], acceleration_, sampling_duration_), -max_cartesian_vel_));
    max_negative_cartesian_vel.push_back(std::max(calculateMaxNegativeVel(frame.p.z(), min_cartesian_limits_[i][2], acceleration_, sampling_duration_), -max_cartesian_vel_));


    double q_x, q_y, q_z, q_w;
    frame.M.GetQuaternion(q_x, q_y, q_z, q_w);
    Eigen::Quaterniond current_rotation(q_w, q_x, q_y, q_z);
    Eigen::Matrix3d min_cartesian_mat;
    double min_x, min_y, min_z;
    if(min_cartesian_limits_[i][3]<-6.28){
      min_x=0;
    }
    else{
      min_x=min_cartesian_limits_[i][3];
    }
    if(min_cartesian_limits_[i][4]<-6.28){
      min_y=0;
    }
    else{
      min_y=min_cartesian_limits_[i][4];
    }
    if(min_cartesian_limits_[i][5]<-6.28){
      min_z=0;
    }
    else{
      min_z=min_cartesian_limits_[i][5];
    }
    min_cartesian_mat=Eigen::AngleAxisd(min_z, Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(min_y, Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(min_z, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond quat_limit_min(min_cartesian_mat);
    /*std::cout<<"quaternion min"<<std::endl;
    std::cout<<quat_limit_min.x()<<" "<<quat_limit_min.y()<<" "<<quat_limit_min.z()<<" "<<quat_limit_min.w()<<std::endl;
    std::cout<<"quaternion current"<<std::endl;
    std::cout<<current_rotation.x()<<" "<<current_rotation.y()<<" "<<current_rotation.z()<<" "<<current_rotation.w()<<std::endl;*/
    Eigen::Vector3d rot_dif=quaternionsSubstraction(quat_limit_min, current_rotation);
    //std::cout<<"diff"<<std::endl;
    //std::cout<<rot_dif<<std::endl;

    //std::cout<<"current_rotation"<<rot_dif<<std::endl;

    if(min_cartesian_limits_[i][3]<-6.28){
      max_negative_cartesian_vel.push_back(-max_cartesian_vel_);
    }
    else{
      max_negative_cartesian_vel.push_back(std::max(calculateMaxNegativeVel(rot_dif[0], acceleration_, sampling_duration_), -max_cartesian_vel_));
    }
    if(min_cartesian_limits_[i][4]<-6.28){
      max_negative_cartesian_vel.push_back(-max_cartesian_vel_);
    }
    else{
      max_negative_cartesian_vel.push_back(std::max(calculateMaxNegativeVel(rot_dif[1], acceleration_, sampling_duration_), -max_cartesian_vel_));
    }
    if(min_cartesian_limits_[i][5]<-6.28){
     max_negative_cartesian_vel.push_back(-max_cartesian_vel_);
    }
    else{
      max_negative_cartesian_vel.push_back(std::max(calculateMaxNegativeVel(rot_dif[2], acceleration_, sampling_duration_), -max_cartesian_vel_));
    }
    max_negative_cartesian_vels.push_back(max_negative_cartesian_vel);

    max_positive_cartesian_vel.push_back(std::min(calculateMaxPositiveVel(frame.p.x(), max_cartesian_limits_[i][0], acceleration_, sampling_duration_),max_cartesian_vel_));
    max_positive_cartesian_vel.push_back(std::min(calculateMaxPositiveVel(frame.p.y(), max_cartesian_limits_[i][1], acceleration_, sampling_duration_), max_cartesian_vel_));
    max_positive_cartesian_vel.push_back(std::min(calculateMaxPositiveVel(frame.p.z(), max_cartesian_limits_[i][2], acceleration_, sampling_duration_), max_cartesian_vel_));

    Eigen::Matrix3d max_cartesian_mat;
    double max_x, max_y, max_z;
    if(max_cartesian_limits_[i][3]>6.28){
      max_x=0;
    }
    else{
      max_x=max_cartesian_limits_[i][3];
    }
    if(max_cartesian_limits_[i][4]>6.28){
      max_y=0;
    }
    else{
      max_y=max_cartesian_limits_[i][4];
    }
    if(max_cartesian_limits_[i][5]>6.28){
      max_z=0;
    }
    else{
      max_z=max_cartesian_limits_[i][5];
    }
    max_cartesian_mat=Eigen::AngleAxisd(max_z, Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(max_y, Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(max_z, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond quat_limit_max(max_cartesian_mat);
    /*std::cout<<"quaternion max"<<std::endl;
    std::cout<<quat_limit_max.x()<<" "<<quat_limit_max.y()<<" "<<quat_limit_max.z()<<" "<<quat_limit_max.w()<<std::endl;
    std::cout<<"quaternion current"<<std::endl;
    std::cout<<current_rotation.x()<<" "<<current_rotation.y()<<" "<<current_rotation.z()<<" "<<current_rotation.w()<<std::endl;*/
    rot_dif=quaternionsSubstraction(quat_limit_max, current_rotation);
   // std::cout<<"diff"<<std::endl;
    //std::cout<<rot_dif<<std::endl;

    //std::cout<<"current_rotation"<<rot_dif<<std::endl;

    if(max_cartesian_limits_[i][3]>6.28){
      max_positive_cartesian_vel.push_back(max_cartesian_vel_);
    }
    else{
      max_positive_cartesian_vel.push_back(std::min(calculateMaxPositiveVel(rot_dif[0], acceleration_, sampling_duration_),max_cartesian_vel_));
    }
    if(max_cartesian_limits_[i][4]>6.28){
      max_positive_cartesian_vel.push_back(max_cartesian_vel_);
    }
    else{
      max_positive_cartesian_vel.push_back(std::min(calculateMaxPositiveVel(rot_dif[1], acceleration_, sampling_duration_), max_cartesian_vel_));
    }
    if(max_cartesian_limits_[i][5]>6.28){
     max_positive_cartesian_vel.push_back(max_cartesian_vel_);
    }
    else{
      max_positive_cartesian_vel.push_back(std::min(calculateMaxPositiveVel(rot_dif[2], acceleration_, sampling_duration_), max_cartesian_vel_));
    }
    max_positive_cartesian_vels.push_back(max_positive_cartesian_vel);


  }
  std::vector<std::vector<std::vector<float> > > max_vels;
  max_vels.push_back(max_positive_cartesian_vels);
  max_vels.push_back(max_negative_cartesian_vels);
  return max_vels;
}

void GraspController::publishStatus(Eigen::MatrixXd vels){
  task_priority::TaskPriority_msg msg;
  for(int i=0; i<multitasks_.size(); i++){
    msg.multi_task.push_back(multitasks_[i]->getMsg(vels));
  }
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id="task_priority";
  status_pub_.publish(msg);

}
void GraspController::publishTwist(Eigen::MatrixXd vels, std::vector<float> odom){

  for(int i=0; i<chains_.size(); i++){
    Eigen::MatrixXd jac_cartesian(6, vels.rows());
    jac_cartesian.setZero();
    KDL::Chain chain_odom;
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    chain_odom.addChain(chains_[i]);
    KDL::ChainJntToJacSolver jac_sovler(chain_odom);

    KDL::JntArray q(chain_odom.getNrOfJoints());

    for(int j=0; j<odom.size(); j++){
      q(j)=odom[j];
    }

    for(int j=0; j<chains_[i].getNrOfJoints(); j++){
      q(j+odom.size())=current_joints_[chain_joint_relations_[i][j]];
    }

    KDL::Jacobian jac_chain(chain_odom.getNrOfJoints());
    jac_sovler.JntToJac(q, jac_chain);

    for(int j=0; j<chain_joint_relations_[i].size(); j++){
     jac_cartesian(0,chain_joint_relations_[i][j])=jac_chain(0, j+odom.size());
     jac_cartesian(1,chain_joint_relations_[i][j])=jac_chain(1, j+odom.size());
     jac_cartesian(2,chain_joint_relations_[i][j])=jac_chain(2, j+odom.size());
     jac_cartesian(3,chain_joint_relations_[i][j])=jac_chain(3, j+odom.size());
     jac_cartesian(4,chain_joint_relations_[i][j])=jac_chain(4, j+odom.size());
     jac_cartesian(5,chain_joint_relations_[i][j])=jac_chain(5, j+odom.size());
    }

    Eigen::MatrixXd cartesian_vel=jac_cartesian*vels;

    geometry_msgs::TwistStamped twist;
    twist.header.stamp=ros::Time::now();
    twist.twist.linear.x=cartesian_vel(0,0);
    twist.twist.linear.y=cartesian_vel(1,0);
    twist.twist.linear.z=cartesian_vel(2,0);
    twist.twist.angular.x=cartesian_vel(3,0);
    twist.twist.angular.x=cartesian_vel(4,0);
    twist.twist.angular.x=cartesian_vel(5,0);
    if(i==0){
      effector_twist_.publish(twist);
    }
    else{
      vehicle_twist_.publish(twist);
    }

  }
}

bool GraspController::isInsideLimits(std::vector<float> odom){
  for(int i=0; i<current_joints_.size(); i++){
    if(current_joints_[i]<min_joint_limit_[i] || current_joints_[i]> max_joint_limit_[i]){
     std::cout<<"el joint "<<i<<"fuera del limite"<<std::endl;
     std::cout<<current_joints_[i]<<"  "<<min_joint_limit_[i]<<"  "<<max_joint_limit_[i]<<std::endl;
      return false;
    }
  }

  for(int i=0; i<chains_.size(); i++){
    KDL::Chain chain_odom;
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    chain_odom.addChain(chains_[i]);
    KDL::ChainFkSolverPos_recursive fk(chain_odom);

    KDL::JntArray q(chain_odom.getNrOfJoints());
    for(int j=0; j<odom.size(); j++){
      q(j)=odom[j];

    }
    for(int j=0; j<chains_[i].getNrOfJoints(); j++){
      q(j+odom.size())=current_joints_[chain_joint_relations_[i][j]];
    }

    KDL::Frame frame;
    fk.JntToCart(q, frame);
    double roll, pitch, yaw;
    frame.M.GetEulerZYX(yaw, pitch, roll);

    std::vector<float> current_pose;
    current_pose.push_back(frame.p.x());
    current_pose.push_back(frame.p.y());
    current_pose.push_back(frame.p.z());
    current_pose.push_back(roll);
    current_pose.push_back(pitch);
    current_pose.push_back(yaw);
    //std::cout<<roll<<"   "<<pitch<<"   "<<yaw<<std::endl;

   for(int j=0; j<6; j++){
     if(current_pose[j]<min_cartesian_limits_[i][j] || current_pose[j]>max_cartesian_limits_[i][j]){
       std::cout<<"cadena "<<i<<"cartesian "<<j<<"fuera del limite"<<std::endl;
       std::cout<<current_pose[j]<<"  "<<min_cartesian_limits_[i][j]<<"  "<<max_cartesian_limits_[i][j]<<std::endl;
       return false;
     }
   }

  }
  return true;
}

void GraspController::publishPose(std::vector<float> odom){
  for(int i=0; i<chains_.size(); i++){
    KDL::Chain chain_odom;
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    chain_odom.addChain(chains_[i]);
    KDL::ChainFkSolverPos_recursive fk(chain_odom);

    KDL::JntArray q(chain_odom.getNrOfJoints());
    for(int j=0; j<odom.size(); j++){
      q(j)=odom[j];

    }
    for(int j=0; j<chains_[i].getNrOfJoints(); j++){
      q(j+odom.size())=current_joints_[chain_joint_relations_[i][j]];
    }

    KDL::Frame frame;
    fk.JntToCart(q, frame);

    double x_a, y_a, z_a, w_a;
    frame.M.GetQuaternion(x_a, y_a, z_a, w_a);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp=ros::Time::now();
    pose.pose.position.x=frame.p.x();
    pose.pose.position.y=frame.p.y();
    pose.pose.position.z=frame.p.z();
    pose.pose.orientation.x=x_a;
    pose.pose.orientation.y=y_a;
    pose.pose.orientation.z=z_a;
    pose.pose.orientation.w=w_a;

    if(i==0){
      effector_pose_.publish(pose);
    }
    else{
      vehicle_pose_.publish(pose);
    }

  }
}
void GraspController::publishConstraints(){
  task_priority::HardConstraints_msg constraint;
  for(int i=0; i<max_joint_limit_.size(); i++){
    task_priority::JointConstraint_msg joint_con;
    joint_con.joint=i;
    joint_con.max=max_joint_limit_[i];
    joint_con.min=min_joint_limit_[i];
    constraint.joints.push_back(joint_con);
  }
  for(int i=0; i<chains_.size(); i++){
    for(int j=0; j<6; j++){
      task_priority::CartesianConstraint_msg cart_con;
      cart_con.kinematic_chain=i;
      cart_con.axis=j;
      cart_con.max=max_cartesian_limits_[i][j];
      cart_con.min=min_cartesian_limits_[i][j];
      constraint.cartesian.push_back(cart_con);
    }
  }
  constraint.max_joint_velocity=max_joint_vel_;
  constraint.max_cartesian_velocity=max_cartesian_vel_;

  constraints_pub_.publish(constraint);
}

