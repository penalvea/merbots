#include "merbots_grasp_srv/grasp.hpp"

Grasp::Grasp(ros::NodeHandle nh){
  nh_=nh;
  initialized_=false;
  initialized_opening_=false;
  joint_msg_.name.push_back("Slew");
  joint_msg_.name.push_back("Shoulder");
  joint_msg_.name.push_back("Elbow");
  joint_msg_.name.push_back("JawRotate");
  joint_msg_.name.push_back("JawOpening");
  for(int i=0; i<4; i++){
    joint_msg_.velocity.push_back(0.0);
  }
  joint_msg_.velocity.push_back(-0.3);


   service_=nh.advertiseService("/doGrasping", &Grasp::doGrasping, this);
   service_opening_=nh.advertiseService("/doOpening", &Grasp::doOpening, this);
   service_station_=nh.advertiseService("doGrasping_Station", &Grasp::doGraspingStation, this);
   sensor0_=0;
   sensor1_=0;
   sensor2_=0;
   sensor3_=0;
   sensor0_sub_=nh_.subscribe<geometry_msgs::WrenchStamped>("optoforce/sensor_0", 1, &Grasp::sensor0Callback, this);
   sensor1_sub_=nh_.subscribe<geometry_msgs::WrenchStamped>("optoforce/sensor_1", 1, &Grasp::sensor1Callback, this);
   sensor2_sub_=nh_.subscribe<geometry_msgs::WrenchStamped>("optoforce/sensor_2", 1, &Grasp::sensor2Callback, this);
   sensor3_sub_=nh_.subscribe<geometry_msgs::WrenchStamped>("optoforce/sensor_3", 1, &Grasp::sensor3Callback, this);
}

Grasp::~Grasp(){}

void Grasp::jointsCallback(const sensor_msgs::JointStateConstPtr &msg){
  effort_=msg->effort[0];
  gripper_pose_=msg->position[4];
  initialized_=true;
}
void Grasp::jointsCallbackOpening(const sensor_msgs::JointStateConstPtr &msg){
  effort_opening_=msg->effort[0];
  gripper_pose_opening_=msg->position[4];
  initialized_opening_=true;
}
void Grasp::jointsCallbackGraspStation(const sensor_msgs::JointStateConstPtr &msg){
  effort_station_=msg->effort[0];
  gripper_pose_station_=msg->position[4];
  initialized_station_=true;
}

bool Grasp::doGrasping(merbots_grasp_srv::grasp_srv::Request &req, merbots_grasp_srv::grasp_srv::Response &res){
  initialized_=false;
  joint_sub_=nh_.subscribe<sensor_msgs::JointState>(req.joint_state_topic, 1, &Grasp::jointsCallback, this);
  joint_pub_=nh_.advertise<sensor_msgs::JointState>(req.command_topic, 1);
  while(ros::ok() && !initialized_){
    usleep(1000);
    ros::spinOnce();
  }
  joint_msg_.velocity[4]=-0.3;
  while(effort_<req.grasped_current && gripper_pose_>=0.05 && ros::ok()){
    joint_msg_.header.stamp=ros::Time::now();
    joint_pub_.publish(joint_msg_);
    ros::spinOnce();
    usleep(1000);
  }
  if(effort_>=req.grasped_current){
    res.success=true;
  }
  else{
    joint_msg_.velocity[4]=0.3;
    while(effort_<1.2 && gripper_pose_<=1.3 && ros::ok()){
      joint_msg_.header.stamp=ros::Time::now();
      joint_pub_.publish(joint_msg_);
      ros::spinOnce();
      usleep(1000);
    }
    joint_msg_.velocity[4]=-0.3;
    res.success=false;
  }
  return true;
}

bool Grasp::doOpening(merbots_grasp_srv::open_gripper_srv::Request &req, merbots_grasp_srv::open_gripper_srv::Response &res){
  initialized_opening_=false;
  joint_sub_opening_=nh_.subscribe<sensor_msgs::JointState>(req.joint_state_topic, 1, &Grasp::jointsCallbackOpening, this);
  joint_pub_opening_=nh_.advertise<sensor_msgs::JointState>(req.command_topic, 1);
  while(ros::ok() && !initialized_opening_){
    usleep(1000);
    ros::spinOnce();
  }
  if(gripper_pose_opening_>req.gripper_joint_position){
    joint_msg_.velocity[4]=-0.3;
    while(effort_<req.max_current && gripper_pose_opening_>req.gripper_joint_position && ros::ok()){
      joint_msg_.header.stamp=ros::Time::now();
      joint_pub_opening_.publish(joint_msg_);
      ros::spinOnce();
      usleep(1000);
    }
    if(gripper_pose_opening_>req.gripper_joint_position)
      res.success=false;
  }
  else{
    joint_msg_.velocity[4]=0.3;
    while(effort_<req.max_current && gripper_pose_opening_<req.gripper_joint_position && ros::ok()){
      joint_msg_.header.stamp=ros::Time::now();
      joint_pub_opening_.publish(joint_msg_);
      ros::spinOnce();
      usleep(1000);
    }
    if(gripper_pose_opening_<req.gripper_joint_position)
      res.success=false;
  }
  res.success=true;
  return true;
}


bool Grasp::doGraspingStation(merbots_grasp_srv::grasp_station_srv::Request &req, merbots_grasp_srv::grasp_station_srv::Response &res){
  ros::Rate r(10);
  sensor0_initial_=sensor0_;
  sensor1_initial_=sensor1_;
  sensor2_initial_=sensor2_;
  sensor3_initial_=sensor3_;


  initialized_=false;
  joint_sub_=nh_.subscribe<sensor_msgs::JointState>(req.joint_state_topic, 1, &Grasp::jointsCallbackGraspStation, this);
  joint_pub_=nh_.advertise<sensor_msgs::JointState>(req.command_topic, 1);
  station_pub_=nh_.advertise<auv_msgs::WorldWaypointReq>(req.station_topic, 1);
  auv_msgs::WorldWaypointReq station_msg;
  station_msg.header.frame_id="girona500";
  station_msg.goal.requester="irslab";
  station_msg.goal.id=0;
  station_msg.goal.priority=10;
  station_msg.position.north=req.north;
  station_msg.position.east=req.east;
  station_msg.position.depth=req.depth;
  station_msg.altitude_mode=false;
  station_msg.altitude=0;
  station_msg.orientation.roll=req.roll;
  station_msg.orientation.pitch=req.pitch;
  station_msg.orientation.yaw=req.yaw;

  station_msg.disable_axis.roll=true;
  station_msg.disable_axis.pitch=true;
  station_msg.disable_axis.yaw=true;

  station_msg.position_tolerance.x=0;
  station_msg.position_tolerance.y=0;
  station_msg.position_tolerance.z=0;
  station_msg.orientation_tolerance.roll=0;
  station_msg.orientation_tolerance.pitch=0;
  station_msg.orientation_tolerance.yaw=0;

  while(ros::ok() && !initialized_station_){
    r.sleep();
    ros::spinOnce();
  }
  joint_msg_.velocity[4]=-0.3;
  int current_detected=0;
  while(current_detected<5 && gripper_pose_station_>=0.05 && ros::ok() && !isGrasped()){
    if(effort_station_>req.grasped_current){
      current_detected++;
    }
    joint_msg_.header.stamp=ros::Time::now();
    joint_pub_.publish(joint_msg_);
    station_msg.header.stamp=ros::Time::now();
    station_pub_.publish(station_msg);
    ros::spinOnce();
    r.sleep();

  }
  if(current_detected>=5){
    station_msg.position.depth=1.0;

    std::cout<<"subimos"<<std::endl;

    while (ros::ok()) {
      station_msg.header.stamp=ros::Time::now();
      station_pub_.publish(station_msg);
      ros::spinOnce();
      r.sleep();
    }
    std::cout<<"salimos subir"<<std::endl;

    res.success=true;
  }
  else{
    joint_msg_.velocity[4]=0.3;
    while(effort_station_<1.2 && gripper_pose_station_<=1.3 && ros::ok()){
      joint_msg_.header.stamp=ros::Time::now();
      joint_pub_.publish(joint_msg_);
      station_msg.header.stamp=ros::Time::now();
      station_pub_.publish(station_msg);
      ros::spinOnce();
      r.sleep();
    }
    joint_msg_.velocity[4]=-0.3;
    res.success=false;
  }
  return true;
}

void Grasp::sensor0Callback(const geometry_msgs::WrenchStampedConstPtr &msg){
  sensor0_=msg->wrench.force.z;
}
void Grasp::sensor1Callback(const geometry_msgs::WrenchStampedConstPtr &msg){
  sensor1_=msg->wrench.force.z;
}
void Grasp::sensor2Callback(const geometry_msgs::WrenchStampedConstPtr &msg){
  sensor2_=msg->wrench.force.z;
}
void Grasp::sensor3Callback(const geometry_msgs::WrenchStampedConstPtr &msg){
  sensor3_=msg->wrench.force.z;
}

bool Grasp::isGrasped(){
  return ((std::abs(sensor0_-sensor0_initial_)>100 || std::abs(sensor1_-sensor1_initial_)>100) && (std::abs(sensor2_-sensor2_initial_)>100 || std::abs(sensor3_-sensor3_initial_)>100));
}

