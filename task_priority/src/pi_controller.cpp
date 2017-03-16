#include <task_priority/pi_controller.hpp>

PIController::PIController(std::vector<float> p_values, std::vector<float> i_values){
  p_values_=p_values;
  i_values_=i_values;
  //i_accum_.resize(p_values_.size());
  for(int i=0; i<p_values_.size(); i++){
    error_accum_.push_back(0.0);
    error_.push_back(0.0);
  }
  last_vels_.resize(p_values.size());
  last_joints_.resize(p_values.size());
  init_=false;
}

PIController::~PIController(){}

void PIController::updateController(std::vector<float> odom, std::vector<float> current_joints, ros::Time time){
  Eigen::Transform<float, 3, Eigen::Affine> wMcv=Eigen::Translation3f(odom[0], odom[1], odom[2])*Eigen::AngleAxisf(odom[5], Eigen::Vector3f::UnitZ());
  if(init_){
    float seconds=(time-last_time_).toSec();

    Eigen::Transform<float, 3, Eigen::Affine> lvMcv=wMlv_.inverse()*wMcv;
    Eigen::Vector3f rot_euler=lvMcv.rotation().eulerAngles(0,1,2);
    error_[0]=last_vels_[0]-(lvMcv.translation()[0]/seconds);
    error_accum_[0]+=error_[0];
    error_[1]=last_vels_[1]-(lvMcv.translation()[1]/seconds);
    error_accum_[1]+=error_[1];
    error_[2]=last_vels_[2]-(lvMcv.translation()[2]/seconds);
    error_accum_[2]+=error_[2];
    error_[3]=last_vels_[3]-(rot_euler[2]/seconds);
    error_accum_[3]+=error_[3];


    /*std::cout<<last_vels_[0]<<" ---- "<<lvMcv.translation()[0]/seconds<<std::endl;
    std::cout<<"                                               "<<(lvMcv.translation()[0]/seconds)-last_vels_[0]<<std::endl;
    std::cout<<last_vels_[1]<<" ---- "<<lvMcv.translation()[1]/seconds<<std::endl;
    std::cout<<"                                               "<<(lvMcv.translation()[1]/seconds)-last_vels_[1]<<std::endl;
    std::cout<<last_vels_[2]<<" ---- "<<lvMcv.translation()[2]/seconds<<std::endl;
    std::cout<<"                                               "<<(lvMcv.translation()[2]/seconds)-last_vels_[2]<<std::endl;
    std::cout<<last_vels_[3]<<" ---- "<<rot_euler[2]/seconds<<std::endl;
    std::cout<<"                                               "<<(rot_euler[2]/seconds)-last_vels_[3]<<std::endl;*/

    for(int i=4; i<last_vels_.size(); i++){
      float current_vel=(current_joints[i]-last_joints_[i])/seconds;
      //std::cout<<last_vels_[i]<<" ---- "<<current_vel<<std::endl;
      //std::cout<<"                                               "<<current_vel-last_vels_[i]<<std::endl;
      error_[i]=last_vels_[i]-current_vel;
      error_accum_[i]=error_[i];
    }
  }
  //std::cout<<i_accum_.size()<<std::endl;
  //for(int i=0; i<error_accum_.size(); i++){
   // std::cout<<i_accum_[i]<<std::endl;

  //}
  last_joints_=current_joints;

  last_odom_=odom;

  wMlv_=wMcv;
  init_=true;
}

Eigen::MatrixXd PIController::getVels(Eigen::MatrixXd vels, ros::Time time){
  for(int i=0; i<last_vels_.size(); i++){
    last_vels_[i]=vels(i,0);
    vels(i,0)=vels(i,0)+(p_values_[i]*error_[i])+(i_values_[i]*error_accum_[i]);
  }
  last_time_=time;
  return vels;
}
