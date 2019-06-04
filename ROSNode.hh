#ifndef MOBILESIM_ROSNODE_HH
#define MOBILESIM_ROSNODE_HH

#include "MobileSim.hh"
#include "RobotInterface.hh"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/callback_queue_interface.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"

/** @TODO XXX configurable frame IDs */

class ROSNode : public LogInterface {
private:
  RobotInterface *robot;

  ros::NodeHandle nodeHandle;
  ros::Publisher pose_pub;
  ros::Publisher bumpers_pub; 
  ros::Publisher sonar_pub;
  ros::Publisher sonar_pointcloud2_pub;
  bool publish_sonar, publish_sonar_pointcloud2;
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

  class PublishCallback : public virtual ros::CallbackInterface {
    ROSNode *target;
  public:
    PublishCallback(ROSNode *rn) : target(rn) {}
    virtual ros::CallbackInterface::CallResult call() {
      target->publish();
      return CallResult::TryAgain;
    }
    virtual bool ready() {
      return true; // TODO could get hint from simulation loop in main about whether it's time, but this is not neccesary as long as that main loop calls ros::spinOnce() to determine interval.
    }
  };

 // PublishCallback publishCB;
  //boost::shared_ptr<PublishCallback> publishCBPtr;

  void cmdvel_cb(const geometry_msgs::TwistConstPtr &);

public:
  ROSNode(RobotInterface *r, MobileSim::Options* opts);
  ~ROSNode();
  bool start();
  void publish(); ///< Normally called automatically by ROS callback spin loop
};

#endif
