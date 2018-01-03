#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <string>
#include <boost/lexical_cast.hpp>
#include "task_priority/controller.hpp"
#include "task_priority/grasp_planning_controller.hpp"


int main(int argc, char **argv){
  ros::init(argc, argv, "multiple_task_priority_control");
  ros::NodeHandle nh;

  std::vector<std::string> prefixes;
  prefixes.push_back("first_stage/");
  prefixes.push_back("second_stage/");
  prefixes.push_back("third_stage/");

  std::vector<float> joints(8,0);
  joints[6]=2,15;
  joints[8]=1.57;

  for(int prefix=0; prefix<prefixes.size(); prefix++){

    double acceleration, max_joint_vel, max_cartesian_vel, sampling_duration;
    nh.getParam(prefixes[prefix]+"acceleration", acceleration);

    ROS_INFO("Acceleration: %f", acceleration);

    nh.getParam(prefixes[prefix]+"max_joint_vel", max_joint_vel);
    ROS_INFO("Max joint vel: %f", max_joint_vel);
    nh.getParam(prefixes[prefix]+"max_cartesian_vel", max_cartesian_vel);
    ROS_INFO("Max cartesian vel: %f", max_cartesian_vel);
    nh.getParam(prefixes[prefix]+"sampling_duration", sampling_duration);
    ROS_INFO("Sampling duration: %f", sampling_duration);

    std::vector<float> odom;
    nh.getParam(prefixes[prefix]+"odometry", odom);
    if(odom.size()!=6){
      ROS_ERROR("Odometry must have 6 values");
    }

    bool simulation=true;
    nh.getParam(prefixes[prefix]+"simulation", simulation);
    ROS_INFO("Simulation: %d", simulation);

    bool grasp_planning=false;
    nh.getParam(prefixes[prefix]+"grasp_planning", grasp_planning);
    ROS_INFO("Grasp_planning: %d", grasp_planning);


    int n_joints;
    nh.getParam(prefixes[prefix]+"n_joints", n_joints);

    std::vector<float> max_joint_limit, min_joint_limit;
    nh.getParam(prefixes[prefix]+"max_joint_limit", max_joint_limit);
    nh.getParam(prefixes[prefix]+"min_joint_limit", min_joint_limit);

    if(max_joint_limit.size()!=n_joints){
      ROS_ERROR("max joint limit must have %d limits", n_joints);
      return -1;
    }
    if(min_joint_limit.size()!=n_joints){
      ROS_ERROR("min joint limit must have %d limits", n_joints);
      return -1;
    }

    std::vector<float> p_values, i_values, d_values;
    nh.getParam(prefixes[prefix]+"PID_p_values", p_values);
    nh.getParam(prefixes[prefix]+"PID_i_values", i_values);
    nh.getParam(prefixes[prefix]+"PID_d_values", d_values);
    if(p_values.size()!=n_joints){
      ROS_ERROR("PID: %d p values needed", n_joints);
      return -1;
    }
    if(i_values.size()!=n_joints){
      ROS_ERROR("PID: %d i values needed", n_joints);
      return -1;
    }
    if(d_values.size()!=n_joints){
      ROS_ERROR("PID: %d d values needed", n_joints);
      return -1;
    }


    std::string arm_joint_state_topic, arm_joint_command_topic, vehicle_tf, world_tf, vehicle_command_topic;
    nh.getParam(prefixes[prefix]+"arm_joint_state_topic", arm_joint_state_topic);
    ROS_INFO("Joint State Topic: %s", arm_joint_state_topic.c_str());
    nh.getParam(prefixes[prefix]+"arm_joint_command_topic", arm_joint_command_topic);
    ROS_INFO("Command Joint Topic: %s", arm_joint_state_topic.c_str());
    nh.getParam(prefixes[prefix]+"vehicle_tf", vehicle_tf);
    ROS_INFO("Vehicle tf: %s", vehicle_tf.c_str());
    nh.getParam(prefixes[prefix]+"world_tf", world_tf);
    ROS_INFO("World tf: %s", world_tf.c_str());
    nh.getParam(prefixes[prefix]+"vehicle_command_topic", vehicle_command_topic);
    ROS_INFO("Command Vehicle Topic: %s", vehicle_command_topic.c_str());


    ////// Chains
    ///
    ///

    std::vector<std::string> chain_names;
    nh.getParam(prefixes[prefix]+"chain_names", chain_names);

    std::map<std::string, int> chain_id;
    std::vector<KDL::Chain> chains;
    std::vector<std::vector<int> > chain_joint_relations;
    std::vector<std::vector<float> > max_cartesian_limits, min_cartesian_limits;
    for(int i=0; i<chain_names.size(); i++){

      chain_id[chain_names[i]]=i;
      KDL::Chain chain;
      bool defined=false;
      int cont=1;
      while(!defined){
        XmlRpc::XmlRpcValue definition;
        if(nh.getParam(prefixes[prefix]+chain_names[i]+"/joint_"+boost::lexical_cast<std::string>(cont), definition)){
          cont++;
          if( definition.size()!=1 && definition.size()!=5){

            ROS_ERROR("%s must have 1 or 5 elements", (chain_names[i]+"/joint_"+boost::lexical_cast<std::string>(cont)).c_str());
            return -1;
          }
          std::string joint_mov=static_cast<std::string>(definition[0]);
          if(definition.size()==1){
            if(joint_mov=="TransX"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
            }
            else if(joint_mov=="TransY"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
            }
            else if(joint_mov=="TransZ"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
            }
            else if(joint_mov=="RotX"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
            }
            else if(joint_mov=="RotY"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
            }
            else if(joint_mov=="RotZ"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
            }
            else{
              ROS_ERROR("%s: incorrect rotation and translation", (chain_names[i]+"/joint_"+boost::lexical_cast<std::string>(cont)).c_str());
              return -1;
            }
          }
          else{
            double theta, d, a, alpha;
            a=static_cast<double>(definition[1]);
            alpha=static_cast<double>(definition[2]);
            d=static_cast<double>(definition[3]);
            theta=static_cast<double>(definition[4]);
            if(joint_mov=="TransX"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX), KDL::Frame().DH(a, alpha, d, theta)));
            }
            else if(joint_mov=="TransY"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY), KDL::Frame().DH(a, alpha, d, theta)));
            }
            else if(joint_mov=="TransZ"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ), KDL::Frame().DH(a, alpha, d, theta)));
            }
            else if(joint_mov=="RotX"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame().DH(a, alpha, d, theta)));
            }
            else if(joint_mov=="RotY"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame().DH(a, alpha, d, theta)));
            }
            else if(joint_mov=="RotZ"){
              chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH(a, alpha, d, theta)));
            }
            else{
              ROS_ERROR("%s: incorrect rotation and translation", (chain_names[i]+"/joint_"+boost::lexical_cast<std::string>(cont)).c_str());
              return -1;
            }

          }
        }
        else{
          defined=true;
          chains.push_back(chain);
          std::vector<int> chain_joint_relation;
          if(!nh.getParam(prefixes[prefix]+chain_names[i]+"/chain_joint_relation", chain_joint_relation)){
            ROS_ERROR("Every chain need the chain_joint_relation");
            return -1;
          }
          chain_joint_relations.push_back(chain_joint_relation);

          std::vector<float> max_cartesian_limit, min_cartesian_limit;
          if(!nh.getParam(prefixes[prefix]+chain_names[i]+"/max_cartesian_limit", max_cartesian_limit)){
            ROS_ERROR("Every chain need the max_cartesian_limit");
            return -1;
          }

          max_cartesian_limits.push_back(max_cartesian_limit);
          if(!nh.getParam(prefixes[prefix]+chain_names[i]+"/min_cartesian_limit", min_cartesian_limit)){
            ROS_ERROR("Every chain need the min_cartesian_limit");
            return -1;
          }
          min_cartesian_limits.push_back(min_cartesian_limit);

          ROS_INFO("chain %s added: %d joints", chain_names[i].c_str(), cont-1);
        }
      }

    }


    ////// MultiTask
    ///
    ///

    std::vector<std::string> multitask_priority;
    nh.getParam(prefixes[prefix]+"multitask_priority", multitask_priority);


    std::vector<MultiTaskPtr> multitasks;
    for(int i=0; i< multitask_priority.size(); i++){
      std::vector<std::string> task_names;

      nh.getParam(prefixes[prefix]+multitask_priority[i]+"/tasks", task_names);
      ROS_INFO("Multitask: %s", multitask_priority[i].c_str());
      std::vector<std::vector<int> > joint_priority;

      bool finished=false;
      int cont=1;
      while(!finished){
        std::vector<int> individual_joint_priority;
        if(nh.getParam(prefixes[prefix]+multitask_priority[i]+"/joint_priority_"+boost::lexical_cast<std::string>(cont), individual_joint_priority)){
          joint_priority.push_back(individual_joint_priority);
          cont++;
        }
        else{
          finished=true;
        }

      }
      ROS_INFO("    %d joint_priorities", cont-1);
      std::vector<TaskPtr> tasks;
      for(int j=0; j<task_names.size(); j++){
        ROS_INFO("    Task: %s", task_names[j].c_str());
        std::string task_chain;
        nh.getParam(prefixes[prefix]+task_names[j]+"/chain", task_chain);
        ROS_INFO("        Chain: %s", task_chain.c_str());
        bool cartesian;
        nh.getParam(prefixes[prefix]+task_names[j]+"/cartesian", cartesian);
        ROS_INFO("        Cartesian: %d", cartesian);
        std::vector<int> task_mask_cartesian, task_mask_joint;
        nh.getParam(prefixes[prefix]+task_names[j]+"/mask_cartesian", task_mask_cartesian);
        std::string msg="";
        for(int k=0; k<task_mask_cartesian.size(); k++){
          msg+=boost::lexical_cast<std::string>(task_mask_cartesian[k])+" ";
        }
        ROS_INFO("        Mask Cartesian: %s", msg.c_str());
        nh.getParam(prefixes[prefix]+task_names[j]+"/mask_joint", task_mask_joint);
        msg="";
        for(int k=0; k<task_mask_joint.size(); k++){
          msg+=boost::lexical_cast<std::string>(task_mask_joint[k])+" ";
        }
        ROS_INFO("        Mask Joint: %s", msg.c_str());
        std::string goal_type;
        nh.getParam(prefixes[prefix]+task_names[j]+"/goal_type", goal_type);
        GoalPtr goal;
        if(goal_type=="Fixed"){
          ROS_INFO("        Goal: Fixed");
          std::vector<float> desired_pose;
          nh.getParam(prefixes[prefix]+task_names[j]+"/goal/desired_pose", desired_pose);
          Eigen::MatrixXd desired_pose_eigen(6,1);
          for(int k=0; k<6; k++){
            desired_pose_eigen(k,0)=desired_pose[k];
          }
          msg="";
          for(int k=0; k<desired_pose.size(); k++){
            msg+=boost::lexical_cast<std::string>(desired_pose[k])+" ";
          }
          ROS_INFO("            Desired Pose: %s", msg.c_str());
          GoalFixedPosePtr goal_fixed(new GoalFixedPose(desired_pose_eigen, chains[chain_id[task_chain]], task_mask_cartesian, chain_joint_relations[chain_id[task_chain]], max_cartesian_vel, max_joint_vel));
          goal=goal_fixed;
        }
        else if(goal_type=="ROS_Pose"){
          ROS_INFO("        Goal: ROS Pose");
          std::string goal_topic;
          nh.getParam(prefixes[prefix]+task_names[j]+"/goal/topic", goal_topic);
          ROS_INFO("            ROS Node: %s", goal_topic.c_str());
          GoalROSPosePtr goal_ros_pose(new GoalROSPose(chains[chain_id[task_chain]], task_mask_cartesian, goal_topic, nh, chain_joint_relations[chain_id[task_chain]], max_cartesian_vel, max_joint_vel));
          goal=goal_ros_pose;
        }
        else if(goal_type=="ROS_Twist"){
          ROS_INFO("        Goal: ROS Twist");
          std::string goal_topic;
          nh.getParam(prefixes[prefix]+task_names[j]+"/goal/topic", goal_topic);
          ROS_INFO("            ROS Node: %s", goal_topic.c_str());
          GoalROSTwistPtr goal_ros_twist(new GoalROSTwist(task_mask_cartesian, goal_topic, nh, max_cartesian_vel, max_joint_vel));
          goal=goal_ros_twist;
        }
        else if(goal_type=="Joints"){
          ROS_INFO("        Goal: Joints");
          std::vector<float> joints_position;
          nh.getParam(prefixes[prefix]+task_names[j]+"/goal/joints_position", joints_position);
          msg="";
          for(int k=0; k<joints_position.size(); k++){
            msg+=boost::lexical_cast<std::string>(joints_position[k])+" ";
          }
          ROS_INFO("            Desired Joints Position: %s", msg.c_str());
          GoalJointsPositionPtr goal_joint(new GoalJointsPosition(joints_position, task_mask_joint, max_joint_vel));
          goal=goal_joint;
        }
        else if(goal_type=="JointsROS"){
          ROS_INFO("        Goal: Joints ROS");
          std::string joints_topic;
          nh.getParam(prefixes[prefix]+task_names[j]+"/goal/joints_topic", joints_topic);
          ROS_INFO("            Desired Joints Topic: %s", joints_topic.c_str());
          GoalROSJointsStatePtr goal_joint_ros(new GoalROSJointsState(joints_topic, nh));
          goal=goal_joint_ros;

        }
        else if(goal_type=="Grasp"){
          ROS_INFO("        Goal: Grasp");
          std::string goal_topic;
          nh.getParam(prefixes[prefix]+task_names[j]+"/goal/topic", goal_topic);
          ROS_INFO("            ROS Node: %s", goal_topic.c_str());
          bool force_sensor;
          float max_force;
          std::string force_sensor_topic, set_zero_srv;
          nh.getParam(prefixes[prefix]+task_names[j]+"/goal/force_sensor/active", force_sensor);
          ROS_INFO("            Force sensor: %d", force_sensor);
          if(force_sensor){
            nh.getParam(prefixes[prefix]+task_names[j]+"/goal/force_sensor/topic", force_sensor_topic);
            ROS_INFO("                Topic: %s", force_sensor_topic.c_str());
            nh.getParam(prefixes[prefix]+task_names[j]+"/goal/force_sensor/max_force", max_force);
            ROS_INFO("                Max force: %f", max_force);
            nh.getParam(prefixes[prefix]+task_names[j]+"/goal/force_sensor/set_zero_srv", set_zero_srv);
            ROS_INFO("                Service Set Zero: %s", set_zero_srv.c_str());
          }
          else{
            force_sensor_topic="";
            max_force=0;
            set_zero_srv="";
          }
          GoalGraspPtr goal_ros_pose(new GoalGrasp(chains[chain_id[task_chain]], task_mask_cartesian, goal_topic, nh, chain_joint_relations[chain_id[task_chain]], max_cartesian_vel, max_joint_vel, arm_joint_state_topic, arm_joint_command_topic, force_sensor, force_sensor_topic, max_force, set_zero_srv));
          goal=goal_ros_pose;
        }
        else{
          ROS_ERROR("Goal must be Fixed, ROS_Pose, ROS_Twist or Joints");
        }
        bool frame_inertial;
        nh.getParam(prefixes[prefix]+task_names[j]+"/frame_inertial", frame_inertial);
        ROS_INFO("        Frame Inertial: %d", frame_inertial);
        TaskPtr task(new Task(chains[chain_id[task_chain]],task_mask_cartesian, n_joints, task_mask_joint, chain_joint_relations[chain_id[task_chain]], goal, frame_inertial, cartesian, task_names[j] ));
        tasks.push_back(task);

      }
      MultiTaskPtr multi(new MultiTask(tasks, chains, chain_joint_relations, joint_priority, max_joint_vel, multitask_priority[i]));
      multitasks.push_back(multi);
    }



      GraspControllerPtr grasp_controller(new GraspController(multitasks, n_joints, max_joint_limit, min_joint_limit, max_cartesian_limits, min_cartesian_limits, max_cartesian_vel, acceleration, max_joint_vel, sampling_duration, nh, arm_joint_state_topic, arm_joint_command_topic, vehicle_tf, world_tf, vehicle_command_topic, chains, chain_joint_relations, simulation, p_values, i_values, d_values));

      std::vector<int> important_tasks;
      if(prefix==0){
        important_tasks.push_back(1);
        important_tasks.push_back(2);
      }
      if(prefix==1){
        important_tasks.push_back(1);
        important_tasks.push_back(2);
      }
      if(prefix==2){
        important_tasks.push_back(
1);      }
      joints=grasp_controller->getSolution(joints, odom, important_tasks);

      std::cout<<"Salgo de la iteracion "<<prefixes[prefix]<<std::endl;
      for(int jo=0; jo<joints.size(); jo++){
        std::cout<<joints[jo]<<" ";
      }
      std::cout<<std::endl;;
      //sleep(5);


  }
  return 0;
}

