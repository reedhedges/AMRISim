#ifndef MOBILESIM_ROSNODE_HH
#define MOBILESIM_ROSNODE_HH

#include "MobileSim.hh"

/** @TODO configurable frame IDs */

class ROSNode : public LogInterface {
private:
  RobotInterface *robot;

  ros::NodeHandle nodeHandle;
  ros::Publisher pose_pub;
  ros::Publisher bumpers_pub; 
  ros::Publisher sonar_pub;
  ros::Publisher sonar_pointcloud2_pub;
  ros::Publisher motors_state_pub;
  std_msgs::Bool motors_state;
  bool published_motors_state;
  ros::Subscriber cmdvel_sub;
  ros::ServiceServer enable_srv;
  ros::ServiceServer disable_srv;
  bool enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  //for odom->base_link transform
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  
  std::string frame_id_odom;
  std::string frame_id_base_link;
  std::string frame_id_bumper;
  std::string frame_id_sonar;

  class PublishCallback : public virtual ros::CallbackIntetrface {
    ROSNode *target;
  public:
    PublishCallback(ROSNode *rn) : target(rn) {}
    virtual CallResult call() {
      target->publish();
    }
    virtual bool ready() {
      return true; // TODO could get hint from simulation loop in main about whether it's time, but this is not neccesary as long as that main loop calls ros::spinOnce() to determine interval.
    }
  };

  PublishCallback publishCB;

public:
  ROSNode(StageInterface *_stageInterface, MobileSim::Options* opts);
  ~ROSNode();
  bool start();
  void publish(); ///< Normally called automatically by ROS callback spin loop
};

#endif
