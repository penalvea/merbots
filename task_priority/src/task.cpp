#include "task_priority/task.hpp"


Task::Task(const KDL::Chain &chain, std::vector<int> mask_cartesian, int n_joints, std::vector<int> mask_joint, std::vector<int> chain_joint_relation, GoalPtr goal, bool frame_inertial, bool cartesian, std::string name){
  active_=true;
  chain_=chain;
  chain_joint_relation_=chain_joint_relation;
  mask_cartesian_=mask_cartesian;
  mask_joint_=mask_joint;
  jac_.reset(new CartesianJacobian(chain, n_joints, chain_joint_relation, mask_cartesian, mask_joint, frame_inertial));
  jac_joint_.reset(new JointJacobian(n_joints, chain_joint_relation, mask_joint));
  task_velocity_.reset(new CartesianTaskVelocity());
  goal_=goal;
  cartesian_=cartesian;
  name_=name;

}

Task::~Task(){}

void Task::activate(){
  active_=true;
}
void Task::deactivate(){
  active_=false;
}
bool Task::isActive(){
  return active_;
}


Eigen::MatrixXd Task::calculateCartesianError(Eigen::MatrixXd current_joint_vel, std::vector<float> joints, std::vector<float> odom){
  /*std::cout<<"current direction****"<<std::endl; */
  /*std::cout<<jac_->getJacNoMask()*current_joint_vel<<std::endl; */
  /*std::cout<<"goal---"<<std::endl; */
  /*std::cout<<goal_->getGoal(joints, odom)<<std::endl; */
  /*std::cout<<"-----------------"<<std::endl; */

  Eigen::MatrixXd vel_error=task_velocity_->calculateCartesianVelocity(jac_->getJacNoMask()*current_joint_vel, goal_->getGoal(joints, odom));
  /*std::cout<<"vel error"<<std::endl; */
  /*std::cout<<vel_error<<std::endl; */
  /*std::cout<<"-----------------"<<std::endl; */

  for(int i=0; i<vel_error.rows(); i++){
    if(mask_cartesian_[i]==0){

      vel_error(i,0)=0;
        }
  }
  return vel_error;

}
Eigen::MatrixXd Task::calculateJointError(Eigen::MatrixXd current_joint_vel, std::vector<float> joints, std::vector<float> odom){
  Eigen::MatrixXd vel_error=task_velocity_->calculateJointVelocity( current_joint_vel, goal_->getGoal(joints, odom));

  for(int i=0; i<vel_error.rows(); i++){
    if(mask_joint_[i]==0){
      vel_error(i,0)=0;
    }
  }

  return vel_error;
}

Eigen::MatrixXd Task::getJacobian(){
  if(cartesian_)
    return jac_->getJac();
  else
    return jac_joint_->getJac();
}
Eigen::MatrixXd Task::getJacobianNoMask(){
  if(cartesian_)
    return jac_->getJacNoMask();
  else
    return jac_joint_->getJacNoMask();
}
void Task::calculateJacobian(std::vector<float> current_joints, std::vector<float> odom){
  jac_->setOdom(odom);

  jac_->calculateJac(current_joints);
  jac_joint_->calculateJac(current_joints);
}


bool Task::goalInitialized(){
  return goal_->getInitialized();
}

bool Task::isCartesian(){
  return cartesian_;
}

task_priority::Task_msg Task::getMsg(Eigen::MatrixXd vels){
  task_priority::Task_msg msg;
  msg.task_name=name_;
  if(cartesian_)
    msg.error=goal_->getMsg(jac_->getJacNoMask()*vels, mask_cartesian_);
  else
    msg.error=goal_->getMsg(vels, mask_joint_);

  return msg;

}



MultiTask::MultiTask(std::vector<TaskPtr> tasks, std::vector<KDL::Chain> chains, std::vector<std::vector<int> > chain_joint_relations, std::vector<std::vector<int> > joints_priority, float max_joint_vel, std::string name){
  tasks_=tasks;
  chains_=chains;
  chain_joint_relations_=chain_joint_relations;
  joints_priority_=joints_priority;
  name_=name;
  max_joint_vel_=max_joint_vel;

}
MultiTask::~MultiTask(){}



Eigen::MatrixXd MultiTask::pinvMat(Eigen::MatrixXd matrix){
  double epsilon=std::numeric_limits<double>::epsilon();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance=epsilon*std::max(matrix.cols(), matrix.rows())*svd.singularValues().array().abs()(0);
  return svd.matrixV()*(svd.singularValues().array().abs()>tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal()*svd.matrixU().adjoint();

}

void MultiTask::setCurrentJoints(std::vector<float> joints){
  current_joints_=joints;
}

void MultiTask::calculateJacobians(Eigen::MatrixXd T_k_complete){
  for(int i=0; i<tasks_.size(); i++){
    tasks_[i]->calculateJacobian(current_joints_, odom_);
  }

  J_k_=tasks_[0]->getJacobian();
  J_k_no_mask_=tasks_[0]->getJacobianNoMask();

  for(int i=1; i<tasks_.size(); i++){
    Eigen::MatrixXd aux=J_k_;
    Eigen::MatrixXd aux_no_mask=J_k_no_mask_;
    Eigen::MatrixXd J_k_i=tasks_[i]->getJacobian();
    Eigen::MatrixXd J_k_i_no_mask=tasks_[i]->getJacobianNoMask();
    J_k_.resize(aux.rows()+J_k_i.rows(), aux.cols());
    J_k_<<aux,J_k_i;
    J_k_no_mask_.resize(aux_no_mask.rows()+J_k_i_no_mask.rows(), aux_no_mask.cols());
    J_k_no_mask_<<aux_no_mask,J_k_i_no_mask;
  }

  T_k_complete_.resize(T_k_complete.rows()+J_k_.rows(),J_k_.cols());
  T_k_complete_<<J_k_,T_k_complete;


}

void MultiTask::adaptJacobiansTask(std::vector<int> joints_active){
  Eigen::MatrixXd T_k_complete_task=T_k_complete_;
  Eigen::MatrixXd J_k_task=J_k_;
  /*std::cout<<T_k_complete_task<<std::endl; */
  /*std::cout<<"-------------------"<<std::endl; */
  /*std::cout<<J_k_task<<std::endl; */

  for(int j=0; j<joints_active.size(); j++){
    if(joints_active[j]==0){
      for(int i=0; i<J_k_task.rows(); i++){
        T_k_complete_task(i,j)=0;
        J_k_task(i,j)=0;
      }
    }
  }




  Eigen::MatrixXd T_k_complete_task_inverse=pinvMat(T_k_complete_task);
  T_k_.resize(J_k_.cols(), J_k_.rows());
  for(int i=0; i<T_k_.rows(); i++){
    for(int j=0; j<T_k_.cols(); j++){
      T_k_(i,j)=T_k_complete_task_inverse(i,j);
    }
  }
  /*std::cout<<"Jacobiana"<<std::endl; */
  /*std::cout<<J_k_<<std::endl; */
  /*std::cout<<"T_k_complete"<<std::endl; */
  /*std::cout<<T_k_complete_<<std::endl; */
  /*std::cout<<"T_k_complete_task"<<std::endl; */
  /*std::cout<<T_k_complete_task<<std::endl; */
  /*std::cout<<"T_k_complete_task_inverse"<<std::endl; */
  /*std::cout<<T_k_complete_task_inverse<<std::endl; */
  /*std::cout<<"T_k_"<<std::endl; */
  /*std::cout<<T_k_<<std::endl; */

J_k_task_=J_k_task;




  J_null_inv_=T_k_*(pinvMat(J_k_task*T_k_));
  /*std::cout<<"J_null_inv_"<<std::endl; */
  /*std::cout<<J_null_inv_<<std::endl; */

}

Eigen::MatrixXd MultiTask::calculateError(Eigen::MatrixXd last_vel){
  Eigen::MatrixXd error;
  if(tasks_[0]->isCartesian()){
    error=tasks_[0]->calculateCartesianError(last_vel, current_joints_, odom_);
  }
  else{
    error=tasks_[0]->calculateJointError(last_vel, current_joints_, odom_);
  }
  Eigen::MatrixXd aux;
  for(int i=1; i<tasks_.size(); i++){
    aux=error;
    Eigen::MatrixXd task_error;
    if(tasks_[i]->isCartesian()){
      task_error=tasks_[i]->calculateCartesianError(last_vel, current_joints_, odom_);
    }
    else{
      task_error=tasks_[i]->calculateJointError(last_vel, current_joints_, odom_);
    }
    error.resize(aux.rows()+task_error.rows(), task_error.cols());
    error<<aux,task_error;
  }
  return error;
}

Eigen::MatrixXd MultiTask::calculateJointsVel(Eigen::MatrixXd error, std::vector<int> joints_active){
/*std::cout<<"J_null_inv_-------------------------------------------------"<<std::endl;
std::cout<<J_null_inv_<<std::endl;
std::cout<<"error"<<std::endl;
std::cout<<error<<std::endl;*/
  return J_null_inv_*error;
}

Eigen::MatrixXd MultiTask::calculateJointsVelNoNull(Eigen::MatrixXd error, std::vector<int> joints_active){
  /*std::cout<<"La jacobiana usada es"<<std::endl; */
  /*std::cout<<J_k_task_<<std::endl; */
  /*std::cout<<"Y su inversa"<<std::endl; */
  /*std::cout<<pinvMat(J_k_task_)<<std::endl; */


  return pinvMat(J_k_task_)*error;
}

Eigen::MatrixXd MultiTask::nextVelModified(Eigen::MatrixXd last_vel, Eigen::MatrixXd error){

  std::vector<float> modifications(J_null_inv_.rows());
  for(int i=0; i<modifications.size(); i++){
    modifications[i]=1;
  }
  bool change=true;
  Eigen::MatrixXd next_vel;
  while(change){
    change=false;
    Eigen::MatrixXd J_null_inv_aux(J_null_inv_.rows(), J_null_inv_.cols());
    for(int i=0; i<J_null_inv_.rows(); i++){
      for(int j=0; j<J_null_inv_.cols(); j++){
        J_null_inv_aux(i,j)=J_null_inv_(i,j)*modifications[i];
      }
    }
    next_vel=last_vel+(J_null_inv_aux*error);

   /* /*std::cout<<"Error"<<std::endl; */
    /*std::cout<<error<<std::endl; */
    /*std::cout<<"J_null_inv_"<<std::endl; */
    /*std::cout<<J_null_inv_<<std::endl; */
    /*std::cout<<"J_null_inv_aux"<<std::endl; */
    /*std::cout<<J_null_inv_aux<<std::endl; */

    /*std::cout<<"last_vel"<<std::endl; */
    /*std::cout<<last_vel<<std::endl; */
    /*std::cout<<"next_vel"<<std::endl; */
    /*std::cout<<next_vel<<std::endl; */





    for(int i=0; i<next_vel.rows(); i++){
      if(next_vel(i,0)>max_positive_joint_vel_[i]+0.0001){
        change=true;
        modifications[i]=modifications[i]-0.01;
        //std::cout<<"Joint "<<i<<"mayor que max_positive"<<std::endl;
        //std::cout<<next_vel(i,0)<<"    ----->  "<<max_positive_joint_vel_[i]<<std::endl;
      }
      else if(next_vel(i,0)<max_negative_joint_vel_[i]-0.0001){
        change=true;
        modifications[i]=modifications[i]-0.01;
        //std::cout<<"Joint "<<i<<"menor que max_negative"<<std::endl;
        //std::cout<<next_vel(i,0)<<"    ----->  "<<max_negative_joint_vel_[i]<<std::endl;
      }
    }

  }

  /*std::cout<<"Salgo de la funcion nueva"<<std::endl; */


  return next_vel;
}

Eigen::MatrixXd MultiTask::calculateMultiTaskVel(Eigen::MatrixXd last_vel, Eigen::MatrixXd T_k_complete){
  Eigen::MatrixXd initial_vel=last_vel;
  ros::spinOnce();

  calculateJacobians(T_k_complete);
  std::vector<int> joints_active(current_joints_.size(),0);
  for(int i=0; i<joints_priority_.size(); i++){
    //std::cout<<"Task "<<i<<std::endl;

    for(int j=0; j<joints_priority_[i].size(); j++){
      joints_active[joints_priority_[i][j]]=1;
    }

    adaptJacobiansTask(joints_active);



    Eigen::MatrixXd error=calculateError(last_vel);
   /* std::cout<<"joint Vels for the current task"<<std::endl;
    std::cout<<calculateJointsVel(error, joints_active)<<std::endl;
    std::cout<<"direction with this vels"<<std::endl;
    std::cout<<J_k_*calculateJointsVel(error, joints_active)<<std::endl;
    std::cout<<"--------------------------------------------------"<<std::endl;*/
    Eigen::MatrixXd next_vel=last_vel+calculateJointsVel(error, joints_active);



    //next_vel=limitJointsTask(next_vel);
    //next_vel=limitJoints(next_vel);
    next_vel=limitJointsAndCartesian(next_vel);
    //next_vel=limitVels(next_vel);




    //next_vel=limitCartesian(next_vel);
    //next_vel=limitJoints(next_vel);

    last_vel=next_vel;
    //sleep(5);
  }

  //std::cout<<"Empezan las novedades --------------------------------------------"<<std::endl;
  //std::cout<<"La velocidades al terminar la tarea son"<<std::endl;
  //std::cout<<last_vel<<std::endl;
  Eigen::MatrixXd error=calculateError(last_vel);
  /*std::cout<<"Con estas obtenemos un error de"<<std::endl;
  std::cout<<error<<std::endl;
  std::cout<<"Con las jacobianas sin null space obtenemos unas velocidades de"<<std::endl;
  std::cout<<calculateJointsVelNoNull(error, joints_active)<<std::endl;
  std::cout<<"Lo que lleva a un error de"<<std::endl;
  std::cout<<calculateJointsVelNoNull(error, joints_active)<<std::endl;*/
  Eigen::MatrixXd next_vel=last_vel+calculateJointsVelNoNull(error, joints_active);

  //std::cout<<"Sumando a las velocidades anteriores tenemos"<<std::endl;
  //std::cout<<next_vel<<std::endl;

  next_vel=limitJointsAndCartesianNoNull(next_vel);
  //std::cout<<"Limitamos las velocidades con no null"<<std::endl;
  //std::cout<<next_vel<<std::endl;


  ////Deactivate multitask in case it cannot be accomplished at all
  ///

  Eigen::MatrixXd initial_error=calculateError(initial_vel);
  Eigen::MatrixXd current_error=calculateError(next_vel);
  /*std::cout<<"initial error"<<std::endl; */
  /*std::cout<<initial_error<<std::endl; */
  /*std::cout<<"current error"<<std::endl; */
  /*std::cout<<current_error<<std::endl; */
  /*std::cout<<"--------------"<<std::endl; */
  bool deactivate=true;
  //std::cout<<"errors accomplished"<<std::endl;
  for(int i=0; i<initial_error.rows(); i++){
   // std::cout<<std::abs(initial_error(i,0))<<" ---- "<<std::abs(current_error(i,0))<<std::endl;
    if(std::abs(initial_error(i,0))-std::abs(current_error(i,0))>0.001){
      deactivate=false;
    }
  }
  Eigen::MatrixXd output_vel;
  if(deactivate)
    output_vel=initial_vel;
  else
    output_vel=next_vel;
  return output_vel;
}
void MultiTask::setOdom(std::vector<float> odom){
  odom_=odom;
}
Eigen::MatrixXd MultiTask::getT_k_complete(){
  return T_k_complete_;
}

void MultiTask::setMaxPositiveJointVelocity(std::vector<float> vels){
  max_positive_joint_vel_=vels;
}
void MultiTask::setMaxNegativeJointVelocity(std::vector<float> vels){
  max_negative_joint_vel_=vels;
}
void MultiTask::setMaxPositiveCartesianVelocity(std::vector<std::vector<float> > vels){
  max_positive_cartesian_vel_=vels;
}
void MultiTask::setMaxNegativeCartesianVelocity(std::vector<std::vector<float> > vels){
  max_negative_cartesian_vel_=vels;
}


Eigen::MatrixXd MultiTask::limitJointsTask(Eigen::MatrixXd vels){
  Eigen::MatrixXd jac_joint(vels.rows(), vels.rows()), desired_vel_joint(vels.rows()+J_k_task_.rows(),1);
  jac_joint.setZero();
  for(int i=0; i<vels.rows(); i++){
    jac_joint(i,i)=1;
  }
  desired_vel_joint.setZero();



  bool finished=false;
  std::vector<int> active_joints(vels.rows(),0);
  while(!finished){
    finished=true;

    for(int i=0; i<vels.rows(); i++){
      if(vels(i,0)>max_positive_joint_vel_[i]+0.0001){
        active_joints[i]=1;
        desired_vel_joint(i,0)=max_positive_joint_vel_[i]-vels(i,0);
        finished=false;
        //std::cout<<"Joint "<<i<<"mayor que max_positive"<<std::endl;
        //std::cout<<vels(i,0)<<"    ----->  "<<max_positive_joint_vel_[i]<<std::endl;
      }
      else if(vels(i,0)<max_negative_joint_vel_[i]-0.0001){
        active_joints[i]=1;
        desired_vel_joint(i,0)=max_negative_joint_vel_[i]-vels(i,0);
        finished=true;
        //std::cout<<"Joint "<<i<<"menor que max_negative"<<std::endl;
        //std::cout<<vels(i,0)<<"    ----->  "<<max_negative_joint_vel_[i]<<std::endl;
      }
      else if(active_joints[i]==1){
        desired_vel_joint(i,0)=0;
      }
    }

    Eigen::MatrixXd jac_joint_modified=jac_joint;


    for(int i=0; i<active_joints.size(); i++){
      if(active_joints[i]==0){
        jac_joint_modified.row(i).setZero();
      }
    }

    Eigen::MatrixXd new_T_k_complete(jac_joint_modified.rows()+J_k_task_.rows()+T_k_complete_.rows(), jac_joint_modified.cols());
    new_T_k_complete<<jac_joint_modified,J_k_task_,T_k_complete_;
    /*std::cout<<"new T_k_complete"<<std::endl; */
    /*std::cout<<new_T_k_complete<<std::endl; */
    Eigen::MatrixXd new_T_k_inverse=pinvMat(new_T_k_complete);
    /*std::cout<<"new T_k_inverse"<<std::endl; */
    /*std::cout<<new_T_k_inverse<<std::endl; */
    Eigen::MatrixXd new_T_k(jac_joint_modified.cols(), jac_joint_modified.rows()+J_k_task_.rows());
    for(int i=0; i<new_T_k.rows(); i++){
      for(int j=0; j<new_T_k.cols(); j++){
        new_T_k(i,j)=new_T_k_inverse(i,j);
      }
    }
    /*std::cout<<"new T_k"<<std::endl; */
    /*std::cout<<new_T_k<<std::endl; */

    Eigen::MatrixXd jac(jac_joint_modified.rows()+J_k_task_.rows(), jac_joint_modified.cols());
    jac<<jac_joint_modified, J_k_task_;
    Eigen::MatrixXd desired_vel(desired_vel_joint.rows(),1);
    desired_vel<< desired_vel_joint;



    /*std::cout<<"-----------------------------------------------"<<std::endl; */
    /*std::cout<<"vels"<<std::endl; */
    /*std::cout<<vels<<std::endl; */
    /*std::cout<<"desired vels"<<std::endl; */
    /*std::cout<<desired_vel<<std::endl; */
    /*std::cout<<"jacobian with null space"<<std::endl; */
    /*std::cout<<(new_T_k*(pinvMat(jac*new_T_k)))<<std::endl; */
    /*std::cout<<"joint_velocities"<<std::endl; */

    /*std::cout<<(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel<<std::endl; */

    /*std::cout<<"-----------------------------------------------"<<std::endl; */





    vels=vels+(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel;
    /*std::cout<<"new vel"<<std::endl; */
    /*std::cout<<vels<<std::endl; */
  }
  return vels;
}



Eigen::MatrixXd MultiTask::limitJoints(Eigen::MatrixXd vels){
  Eigen::MatrixXd jac_joint(vels.rows(), vels.rows()), desired_vel_joint(vels.rows(),1);
  jac_joint.setZero();
  for(int i=0; i<vels.rows(); i++){
    jac_joint(i,i)=1;
  }
  desired_vel_joint.setZero();



  bool finished=false;
  std::vector<int> active_joints(vels.rows(),0);
  while(!finished){
    finished=true;

    for(int i=0; i<vels.rows(); i++){
      if(vels(i,0)>max_positive_joint_vel_[i]+0.0001){
        active_joints[i]=1;
        desired_vel_joint(i,0)=max_positive_joint_vel_[i]-vels(i,0);
        finished=false;
        //std::cout<<"Joint "<<i<<"mayor que max_positive"<<std::endl;
        //std::cout<<vels(i,0)<<"    ----->  "<<max_positive_joint_vel_[i]<<std::endl;
      }
      else if(vels(i,0)<max_negative_joint_vel_[i]-0.0001){
        active_joints[i]=1;
        desired_vel_joint(i,0)=max_negative_joint_vel_[i]-vels(i,0);
        finished=true;
        //std::cout<<"Joint "<<i<<"menor que max_negative"<<std::endl;
        //std::cout<<vels(i,0)<<"    ----->  "<<max_negative_joint_vel_[i]<<std::endl;
      }
      else if(active_joints[i]==1){
        desired_vel_joint(i,0)=0;
      }
    }

    Eigen::MatrixXd jac_joint_modified=jac_joint;


    for(int i=0; i<active_joints.size(); i++){
      if(active_joints[i]==0){
        jac_joint_modified.row(i).setZero();
      }
    }

    Eigen::MatrixXd new_T_k_complete(jac_joint_modified.rows()+T_k_complete_.rows(), jac_joint_modified.cols());
    new_T_k_complete<<jac_joint_modified,T_k_complete_;
    /*std::cout<<"new T_k_complete"<<std::endl; */
    /*std::cout<<new_T_k_complete<<std::endl; */
    Eigen::MatrixXd new_T_k_inverse=pinvMat(new_T_k_complete);
    /*std::cout<<"new T_k_inverse"<<std::endl; */
    /*std::cout<<new_T_k_inverse<<std::endl; */
    Eigen::MatrixXd new_T_k(jac_joint_modified.cols(), jac_joint_modified.rows());
    for(int i=0; i<new_T_k.rows(); i++){
      for(int j=0; j<new_T_k.cols(); j++){
        new_T_k(i,j)=new_T_k_inverse(i,j);
      }
    }
    /*std::cout<<"new T_k"<<std::endl; */
    /*std::cout<<new_T_k<<std::endl; */

    Eigen::MatrixXd jac(jac_joint_modified.rows(), jac_joint_modified.cols());
    jac<<jac_joint_modified;
    Eigen::MatrixXd desired_vel(desired_vel_joint.rows(),1);
    desired_vel<< desired_vel_joint;



    /*std::cout<<"-----------------------------------------------"<<std::endl; */
    /*std::cout<<"vels"<<std::endl; */
    /*std::cout<<vels<<std::endl; */
    /*std::cout<<"desired vels"<<std::endl; */
    /*std::cout<<desired_vel<<std::endl; */
    /*std::cout<<"jacobian with null space"<<std::endl; */
    /*std::cout<<(new_T_k*(pinvMat(jac*new_T_k)))<<std::endl; */
    /*std::cout<<"joint_velocities"<<std::endl; */

    /*std::cout<<(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel<<std::endl; */

    /*std::cout<<"-----------------------------------------------"<<std::endl; */





    vels=vels+(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel;
    /*std::cout<<"new vel"<<std::endl; */
    /*std::cout<<vels<<std::endl; */
  }
  return vels;
}


Eigen::MatrixXd MultiTask::limitCartesian(Eigen::MatrixXd vels){

  Eigen::MatrixXd jac_cartesian(chains_.size()*6, vels.rows()),desired_vel_cartesian(chains_.size()*6,1);
  jac_cartesian.setZero();
  desired_vel_cartesian.setZero();

  for(int i=0; i<chains_.size(); i++){
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
    for(int j=0; j<odom_.size(); j++){
      q(j)=odom_[j];
    }
    for(int j=0; j<chains_[i].getNrOfJoints(); j++){
      q(j+odom_.size())=current_joints_[chain_joint_relations_[i][j]];
    }

    KDL::Jacobian jac_chain(chain_odom.getNrOfJoints());
    jac_sovler.JntToJac(q, jac_chain);

    for(int j=0; j<chain_joint_relations_[i].size(); j++){
     jac_cartesian(i*6,chain_joint_relations_[i][j])=jac_chain(0, j+odom_.size());
     jac_cartesian(i*6+1,chain_joint_relations_[i][j])=jac_chain(1, j+odom_.size());
     jac_cartesian(i*6+2,chain_joint_relations_[i][j])=jac_chain(2, j+odom_.size());
     jac_cartesian(i*6+3,chain_joint_relations_[i][j])=jac_chain(3, j+odom_.size());
     jac_cartesian(i*6+4,chain_joint_relations_[i][j])=jac_chain(4, j+odom_.size());
     jac_cartesian(i*6+5,chain_joint_relations_[i][j])=jac_chain(5, j+odom_.size());
    }
  }



  bool finished=false;
  bool limit_cartesian=false;
  std::vector<int> active_cartesians(chains_.size()*6,0);

  while(!finished){
    finished=true;
    limit_cartesian=false;


    Eigen::MatrixXd cartesian_velocity=jac_cartesian*vels;
    for(int i=0; i<chains_.size(); i++){
      for(int j=0; j<6; j++){
          if(cartesian_velocity(i*6+j,0)>max_positive_cartesian_vel_[i][j]+0.0001){
            active_cartesians[i*6+j]=1;
            desired_vel_cartesian(i*6+j,0)=max_positive_cartesian_vel_[i][j]-cartesian_velocity(i*6+j);
            finished=false;
            limit_cartesian=true;

            //std::cout<<"Cartesian chain "<<i<<" axis "<<j<<" mayor que max_positive"<<std::endl;
            //std::cout<<cartesian_velocity(i*6+j)<<"    ----->  "<<max_positive_cartesian_vel_[i][j]<<std::endl;
          }
          else if(cartesian_velocity(i*6+j,0)<max_negative_cartesian_vel_[i][j]-0.0001){
            active_cartesians[i*6+j]=1;
            desired_vel_cartesian(i*6+j,0)=max_negative_cartesian_vel_[i][j]-cartesian_velocity(i*6+j);

            finished=false;
            limit_cartesian=true;

            //std::cout<<"Cartesian chain "<<i<<" axis "<<j<<" menor que max_negative"<<std::endl;
            //std::cout<<cartesian_velocity(i*6+j)<<"    ----->  "<<max_negative_cartesian_vel_[i][j]<<std::endl;
          }
          else if(active_cartesians[i*6+j]==1){
            desired_vel_cartesian(i*6+j,0)=0;
        }
      }
    }



    Eigen::MatrixXd jac_cartesian_modified=jac_cartesian;

    for(int i=0; i<active_cartesians.size(); i++){
      if(active_cartesians[i]==0){
        jac_cartesian_modified.row(i).setZero();
      }
    }

   Eigen::MatrixXd new_T_k_complete(jac_cartesian_modified.rows()+T_k_complete_.rows(), jac_cartesian_modified.cols());
   new_T_k_complete<<jac_cartesian_modified,T_k_complete_;
    Eigen::MatrixXd new_T_k_inverse=pinvMat(new_T_k_complete);
    Eigen::MatrixXd new_T_k(jac_cartesian_modified.cols(), jac_cartesian_modified.rows());
    for(int i=0; i<new_T_k.rows(); i++){
      for(int j=0; j<new_T_k.cols(); j++){
        new_T_k(i,j)=new_T_k_inverse(i,j);
      }
    }

    Eigen::MatrixXd jac(jac_cartesian_modified.rows(), jac_cartesian_modified.cols());
    jac<<jac_cartesian_modified;
    Eigen::MatrixXd desired_vel(desired_vel_cartesian.rows(),1);
    desired_vel<<desired_vel_cartesian, jac_cartesian_modified;







    vels=vels+(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel;
  }


  return vels;
}



Eigen::MatrixXd MultiTask::limitJointsAndCartesian(Eigen::MatrixXd vels){
  Eigen::MatrixXd jac_joint(vels.rows(), vels.rows()), desired_vel_joint(vels.rows(),1);
  jac_joint.setZero();
  for(int i=0; i<vels.rows(); i++){
    jac_joint(i,i)=1;
  }
  desired_vel_joint.setZero();

  Eigen::MatrixXd jac_cartesian(chains_.size()*6, vels.rows()),desired_vel_cartesian(chains_.size()*6,1);
  jac_cartesian.setZero();
  desired_vel_cartesian.setZero();

  for(int i=0; i<chains_.size(); i++){
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
    for(int j=0; j<odom_.size(); j++){
      q(j)=odom_[j];
    }
    for(int j=0; j<chains_[i].getNrOfJoints(); j++){
      q(j+odom_.size())=current_joints_[chain_joint_relations_[i][j]];
    }

    KDL::Jacobian jac_chain(chain_odom.getNrOfJoints());
    jac_sovler.JntToJac(q, jac_chain);

    for(int j=0; j<chain_joint_relations_[i].size(); j++){
     jac_cartesian(i*6,chain_joint_relations_[i][j])=jac_chain(0, j+odom_.size());
     jac_cartesian(i*6+1,chain_joint_relations_[i][j])=jac_chain(1, j+odom_.size());
     jac_cartesian(i*6+2,chain_joint_relations_[i][j])=jac_chain(2, j+odom_.size());
     jac_cartesian(i*6+3,chain_joint_relations_[i][j])=jac_chain(3, j+odom_.size());
     jac_cartesian(i*6+4,chain_joint_relations_[i][j])=jac_chain(4, j+odom_.size());
     jac_cartesian(i*6+5,chain_joint_relations_[i][j])=jac_chain(5, j+odom_.size());
    }
  }



  bool finished=false;
  bool limit_cartesian=false;
  std::vector<int> active_cartesians(chains_.size()*6,0);
  std::vector<int> active_joints(vels.rows(),0);
  while(!finished){
    finished=true;
    limit_cartesian=false;


    Eigen::MatrixXd cartesian_velocity=jac_cartesian*vels;
    for(int i=0; i<chains_.size(); i++){
      for(int j=0; j<6; j++){
          if(cartesian_velocity(i*6+j,0)>max_positive_cartesian_vel_[i][j]+0.0001){
            active_cartesians[i*6+j]=1;
            desired_vel_cartesian(i*6+j,0)=max_positive_cartesian_vel_[i][j]-cartesian_velocity(i*6+j);
            finished=false;
            limit_cartesian=true;
            std::fill(active_joints.begin(), active_joints.end(), 0);
            //std::cout<<"Cartesian chain "<<i<<" axis "<<j<<" mayor que max_positive"<<std::endl;
            //std::cout<<cartesian_velocity(i*6+j)<<"    ----->  "<<max_positive_cartesian_vel_[i][j]<<std::endl;
          }
          else if(cartesian_velocity(i*6+j,0)<max_negative_cartesian_vel_[i][j]-0.0001){
            active_cartesians[i*6+j]=1;
            desired_vel_cartesian(i*6+j,0)=max_negative_cartesian_vel_[i][j]-cartesian_velocity(i*6+j);

            finished=false;
            limit_cartesian=true;
            std::fill(active_joints.begin(), active_joints.end(), 0);
            //std::cout<<"Cartesian chain "<<i<<" axis "<<j<<" menor que max_negative"<<std::endl;
            //std::cout<<cartesian_velocity(i*6+j)<<"    ----->  "<<max_negative_cartesian_vel_[i][j]<<std::endl;
          }
          else if(active_cartesians[i*6+j]==1){
            desired_vel_cartesian(i*6+j,0)=0;
        }
      }
    }

    if(!limit_cartesian){
      for(int i=0; i<vels.rows(); i++){
        if(vels(i,0)>max_positive_joint_vel_[i]+0.0001){

          active_joints[i]=1;
          desired_vel_joint(i,0)=max_positive_joint_vel_[i]-vels(i,0);
          finished=false;
          //std::cout<<"Joint "<<i<<"mayor que max_positive"<<std::endl;
          //std::cout<<vels(i,0)<<"    ----->  "<<max_positive_joint_vel_[i]<<std::endl;
        }
        else if(vels(i,0)<max_negative_joint_vel_[i]-0.0001){

          active_joints[i]=1;
          desired_vel_joint(i,0)=max_negative_joint_vel_[i]-vels(i,0);
          finished=true;
          //std::cout<<"Joint "<<i<<"mayor que max_negative"<<std::endl;
          //std::cout<<vels(i,0)<<"    ----->  "<<max_negative_joint_vel_[i]<<std::endl;
        }
        else if(active_joints[i]==1){
          desired_vel_joint(i,0)=0;
        }
      }
    }

    Eigen::MatrixXd jac_cartesian_modified=jac_cartesian;
    Eigen::MatrixXd jac_joint_modified=jac_joint;

    for(int i=0; i<active_cartesians.size(); i++){
      if(active_cartesians[i]==0){
        jac_cartesian_modified.row(i).setZero();
      }
    }
    for(int i=0; i<active_joints.size(); i++){
      if(active_joints[i]==0){
        jac_joint_modified.row(i).setZero();
      }
    }


   Eigen::MatrixXd new_T_k_complete(jac_cartesian_modified.rows()+jac_joint_modified.rows()+T_k_complete_.rows(), jac_joint_modified.cols());
   new_T_k_complete<<jac_cartesian_modified, jac_joint_modified,T_k_complete_;
   /*std::cout<<"new T_k_complete"<<std::endl; */
   /*std::cout<<new_T_k_complete<<std::endl; */
    Eigen::MatrixXd new_T_k_inverse=pinvMat(new_T_k_complete);
    /*std::cout<<"new T_k_inverse"<<std::endl; */
    /*std::cout<<new_T_k_inverse<<std::endl; */
    Eigen::MatrixXd new_T_k(jac_joint_modified.cols(), jac_cartesian_modified.rows()+jac_joint_modified.rows());
    for(int i=0; i<new_T_k.rows(); i++){
      for(int j=0; j<new_T_k.cols(); j++){
        new_T_k(i,j)=new_T_k_inverse(i,j);
      }
    }
    /*std::cout<<"new T_k"<<std::endl; */
    /*std::cout<<new_T_k<<std::endl; */

    Eigen::MatrixXd jac(jac_cartesian_modified.rows()+jac_joint_modified.rows(), jac_joint_modified.cols());
    jac<<jac_cartesian_modified, jac_joint_modified;
    Eigen::MatrixXd desired_vel(desired_vel_cartesian.rows()+ desired_vel_joint.rows(),1);
    desired_vel<<desired_vel_cartesian, desired_vel_joint;




    /*std::cout<<"-----------------------------------------------"<<std::endl; */
    /*std::cout<<"vels"<<std::endl; */
    /*std::cout<<vels<<std::endl; */
    /*std::cout<<"desired vels"<<std::endl; */
    /*std::cout<<desired_vel<<std::endl; */
    /*std::cout<<"jacobian with null space"<<std::endl; */
    /*std::cout<<(new_T_k*(pinvMat(jac*new_T_k)))<<std::endl; */
    /*std::cout<<"joint_velocities"<<std::endl; */

    /*std::cout<<(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel<<std::endl; */

    /*std::cout<<"-----------------------------------------------"<<std::endl; */


    vels=vels+(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel;
    /*std::cout<<"new vel"<<std::endl; */
    /*std::cout<<vels<<std::endl; */

    char a;
   //std::cin>>a;


  }


  return vels;
}



Eigen::MatrixXd MultiTask::limitJointsAndCartesianNoNull(Eigen::MatrixXd vels){
  Eigen::MatrixXd jac_joint(vels.rows(), vels.rows()), desired_vel_joint(vels.rows(),1);
  jac_joint.setZero();
  for(int i=0; i<vels.rows(); i++){
    jac_joint(i,i)=1;
  }
  desired_vel_joint.setZero();

  Eigen::MatrixXd jac_cartesian(chains_.size()*6, vels.rows()),desired_vel_cartesian(chains_.size()*6,1);
  jac_cartesian.setZero();
  desired_vel_cartesian.setZero();

  for(int i=0; i<chains_.size(); i++){
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
    for(int j=0; j<odom_.size(); j++){
      q(j)=odom_[j];
    }
    for(int j=0; j<chains_[i].getNrOfJoints(); j++){
      q(j+odom_.size())=current_joints_[chain_joint_relations_[i][j]];
    }

    KDL::Jacobian jac_chain(chain_odom.getNrOfJoints());
    jac_sovler.JntToJac(q, jac_chain);

    for(int j=0; j<chain_joint_relations_[i].size(); j++){
     jac_cartesian(i*6,chain_joint_relations_[i][j])=jac_chain(0, j+odom_.size());
     jac_cartesian(i*6+1,chain_joint_relations_[i][j])=jac_chain(1, j+odom_.size());
     jac_cartesian(i*6+2,chain_joint_relations_[i][j])=jac_chain(2, j+odom_.size());
     jac_cartesian(i*6+3,chain_joint_relations_[i][j])=jac_chain(3, j+odom_.size());
     jac_cartesian(i*6+4,chain_joint_relations_[i][j])=jac_chain(4, j+odom_.size());
     jac_cartesian(i*6+5,chain_joint_relations_[i][j])=jac_chain(5, j+odom_.size());
    }
  }



  bool finished=false;
  bool limit_cartesian=false;
  std::vector<int> active_cartesians(chains_.size()*6,0);
  std::vector<int> active_joints(vels.rows(),0);
  while(!finished){
    finished=true;
    limit_cartesian=false;


    Eigen::MatrixXd cartesian_velocity=jac_cartesian*vels;
    for(int i=0; i<chains_.size(); i++){
      for(int j=0; j<6; j++){
          if(cartesian_velocity(i*6+j,0)>max_positive_cartesian_vel_[i][j]+0.0001){
            active_cartesians[i*6+j]=1;
            desired_vel_cartesian(i*6+j,0)=max_positive_cartesian_vel_[i][j]-cartesian_velocity(i*6+j);
            finished=false;
            limit_cartesian=true;
            std::fill(active_joints.begin(), active_joints.end(), 0);
            //std::cout<<"Cartesian chain "<<i<<" axis "<<j<<" mayor que max_positive"<<std::endl;
            //std::cout<<cartesian_velocity(i*6+j)<<"    ----->  "<<max_positive_cartesian_vel_[i][j]<<std::endl;
          }
          else if(cartesian_velocity(i*6+j,0)<max_negative_cartesian_vel_[i][j]-0.0001){
            active_cartesians[i*6+j]=1;
            desired_vel_cartesian(i*6+j,0)=max_negative_cartesian_vel_[i][j]-cartesian_velocity(i*6+j);

            finished=false;
            limit_cartesian=true;
            std::fill(active_joints.begin(), active_joints.end(), 0);
            //std::cout<<"Cartesian chain "<<i<<" axis "<<j<<" menor que max_negative"<<std::endl;
            //std::cout<<cartesian_velocity(i*6+j)<<"    ----->  "<<max_negative_cartesian_vel_[i][j]<<std::endl;
          }
          else if(active_cartesians[i*6+j]==1){
            desired_vel_cartesian(i*6+j,0)=0;
        }
      }
    }

    if(!limit_cartesian){
      for(int i=0; i<vels.rows(); i++){
        if(vels(i,0)>max_positive_joint_vel_[i]+0.0001){

          active_joints[i]=1;
          desired_vel_joint(i,0)=max_positive_joint_vel_[i]-vels(i,0);
          finished=false;
          //std::cout<<"Joint "<<i<<"mayor que max_positive"<<std::endl;
          //std::cout<<vels(i,0)<<"    ----->  "<<max_positive_joint_vel_[i]<<std::endl;
        }
        else if(vels(i,0)<max_negative_joint_vel_[i]-0.0001){

          active_joints[i]=1;
          desired_vel_joint(i,0)=max_negative_joint_vel_[i]-vels(i,0);
          finished=true;
          //std::cout<<"Joint "<<i<<"mayor que max_negative"<<std::endl;
          //std::cout<<vels(i,0)<<"    ----->  "<<max_negative_joint_vel_[i]<<std::endl;
        }
        else if(active_joints[i]==1){
          desired_vel_joint(i,0)=0;
        }
      }
    }

    Eigen::MatrixXd jac_cartesian_modified=jac_cartesian;
    Eigen::MatrixXd jac_joint_modified=jac_joint;

    for(int i=0; i<active_cartesians.size(); i++){
      if(active_cartesians[i]==0){
        jac_cartesian_modified.row(i).setZero();
      }
    }
    for(int i=0; i<active_joints.size(); i++){
      if(active_joints[i]==0){
        jac_joint_modified.row(i).setZero();
      }
    }


   Eigen::MatrixXd new_T_k_complete(jac_cartesian_modified.rows()+jac_joint_modified.rows()+J_k_task_.rows(), jac_joint_modified.cols());
   new_T_k_complete<<jac_cartesian_modified, jac_joint_modified,J_k_task_;
   /*std::cout<<"new T_k_complete"<<std::endl; */
   /*std::cout<<new_T_k_complete<<std::endl; */
    Eigen::MatrixXd new_T_k_inverse=pinvMat(new_T_k_complete);
    /*std::cout<<"new T_k_inverse"<<std::endl; */
    /*std::cout<<new_T_k_inverse<<std::endl; */
    Eigen::MatrixXd new_T_k(jac_joint_modified.cols(), jac_cartesian_modified.rows()+jac_joint_modified.rows());
    for(int i=0; i<new_T_k.rows(); i++){
      for(int j=0; j<new_T_k.cols(); j++){
        new_T_k(i,j)=new_T_k_inverse(i,j);
      }
    }
    /*std::cout<<"new T_k"<<std::endl; */
    /*std::cout<<new_T_k<<std::endl; */

    Eigen::MatrixXd jac(jac_cartesian_modified.rows()+jac_joint_modified.rows(), jac_joint_modified.cols());
    jac<<jac_cartesian_modified, jac_joint_modified;
    Eigen::MatrixXd desired_vel(desired_vel_cartesian.rows()+ desired_vel_joint.rows(),1);
    desired_vel<<desired_vel_cartesian, desired_vel_joint;




    /*std::cout<<"-----------------------------------------------"<<std::endl; */
    /*std::cout<<"vels"<<std::endl; */
    /*std::cout<<vels<<std::endl; */
    /*std::cout<<"desired vels"<<std::endl; */
    /*std::cout<<desired_vel<<std::endl; */
    /*std::cout<<"jacobian with null space"<<std::endl; */
    /*std::cout<<(new_T_k*(pinvMat(jac*new_T_k)))<<std::endl; */
    /*std::cout<<"joint_velocities"<<std::endl; */

    /*std::cout<<(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel<<std::endl; */

    /*std::cout<<"-----------------------------------------------"<<std::endl; */


    vels=vels+(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel;
    /*std::cout<<"new vel"<<std::endl; */
    /*std::cout<<vels<<std::endl; */

    char a;
   //std::cin>>a;


  }


  return vels;
}




Eigen::MatrixXd MultiTask::limitVels(Eigen::MatrixXd vels){
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

bool MultiTask::goalsInitialized(){
  bool initialized=true;
  for(int i=0; i<tasks_.size(); i++){
    initialized= initialized && tasks_[i]->goalInitialized();
  }
  return initialized;
}

task_priority::MultiTask_msg MultiTask::getMsg(Eigen::MatrixXd vels){
  task_priority::MultiTask_msg msg;
  msg.name=name_;
  for(int i=0; i<tasks_.size(); i++){
    msg.tasks.push_back(tasks_[i]->getMsg(vels));
  }
  return msg;
}
