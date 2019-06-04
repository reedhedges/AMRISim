
#include "ROSNode.hh"
#include "RobotInterface.hh"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/callback_queue_interface.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud.h>  //for sonar data
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> // can optionally publish sonar as new type pointcloud2
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"  //for tf::getPrefixParam
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"


// TODO:
// publish simulator pose

ROSNode::ROSNode(RobotInterface *r, MobileSim::Options *opts) :
  LogInterface(r->getRobotName()+" ROS Client Interface Node"),
  robot(r),
  nodeHandle(std::string("~")), // XXX TODO unique names for multiple robots
  publish_sonar(false), publish_sonar_pointcloud2(false),
  published_motors_state(false),
  frame_id_odom("odom"), // XXX TODO prepend model name or other unique ID?
  frame_id_base_link("base_link"),
  frame_id_sonar("sonar")
 //publishCB(this)
 // publishCBPtr(boost::make_shared<PublishCallback, ROSNode>(this))
{
  pose_pub = nodeHandle.advertise<nav_msgs::Odometry>("pose",1000);
  sonar_pub = nodeHandle.advertise<sensor_msgs::PointCloud>("sonar", 50, 
      boost::bind(&RobotInterface::openSonar, robot),
      boost::bind(&RobotInterface::closeSonar, robot));
  sonar_pointcloud2_pub = nodeHandle.advertise<sensor_msgs::PointCloud2>("sonar_pointcloud2", 50,
      boost::bind(&RobotInterface::openSonar, robot),
      boost::bind(&RobotInterface::closeSonar, robot));

  motors_state_pub = nodeHandle.advertise<std_msgs::Bool>("motors_state", 5, true /* (latch) */ );
  motors_state.data = false;
  published_motors_state = false;

  //todo enable_srv = nodeHandle.advertiseService("enable_motors", &ROSNode::enable_motors_cb, this);
  //todo disable_srv = nodeHandle.advertiseService("disable_motors", &ROSNode::disable_motors_cb, this);
 
  // TODO laser(s)

  cmdvel_sub = nodeHandle.subscribe( "cmd_vel", 1, (boost::function <void(const geometry_msgs::TwistConstPtr&)>)
          boost::bind(&ROSNode::cmdvel_cb, this, _1 ));


 
}

ROSNode::~ROSNode()
{
  ros::getGlobalCallbackQueue()->removeByID(42422323); // XXX TODO use correct ID from start()
  robot->disableMotors();
  robot->disconnect();
}

bool ROSNode::start()
{
  robot->enableMotors();
  boost::shared_ptr<PublishCallback> p(new PublishCallback(this));
  ros::getGlobalCallbackQueue()->addCallback(p, 42422323); //XXX TODO use a unique number... how to ask the callback queue for a unique number?
  robot->inform("ROS node '%s' is ready", nodeHandle.getNamespace().c_str());

  // TODO connect/disconnectt on first/last subscriber instead?
  robot->connect();
  return true;
}

void ROSNode::publish()
{
  // all topics and transforms published will have same timestamp
  ros::Time timestamp =  ros::Time::now();

  //if(!robot->havePositionData()) print_debug("no position data");
  if(robot->havePositionData())
  {
    int x, y, theta, vel, rotVel;
    bool stall;
    bool motorsEnabled;

    robot->getMotionState(x, y, theta, vel, rotVel, stall, motorsEnabled);
  

    nav_msgs::Odometry position;
    tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(theta*M_PI/180), tf::Vector3(x/1000.0,
      y/1000.0, 0)), position.pose.pose); //Aria returns pose in mm.
    position.twist.twist.linear.x = vel/1000.0; //Aria returns velocity in mm/s.
    position.twist.twist.linear.y = robot->yspeed()/1000.0;
    position.twist.twist.angular.z = rotVel*M_PI/180.0;
    
    position.header.frame_id = frame_id_odom;
    position.child_frame_id = frame_id_base_link;
    position.header.stamp = timestamp;
    pose_pub.publish(position);

    // TODO also publish simulator pose (not odom)

    //print_debug
    ROS_DEBUG
    ("MobileSim ROSNode: publish: (time %f) pose x: %f, pose y: %f, pose angle: %f; linear vel x: %f, vel y: %f; angular vel z: %f", 
      position.header.stamp.toSec(), 
      (double)position.pose.pose.position.x,
      (double)position.pose.pose.position.y,
      (double)position.pose.pose.orientation.w,
      (double)position.twist.twist.linear.x,
      (double)position.twist.twist.linear.y,
      (double)position.twist.twist.angular.z
    );

    // publishing transform odom->base_link
    odom_trans.header.stamp = timestamp;
    odom_trans.header.frame_id = frame_id_odom;
    odom_trans.child_frame_id = frame_id_base_link;
    
    odom_trans.transform.translation.x = x/1000.0;
    odom_trans.transform.translation.y = y/1000.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta*M_PI/180.0);
    
    odom_broadcaster.sendTransform(odom_trans);

    // publish motors state if changed
    if(motorsEnabled != motors_state.data || !published_motors_state)
    {
      ROS_INFO("MobileSim ROSNode: publishing new motors state %d.", motorsEnabled);
      motors_state.data = motorsEnabled;
      motors_state_pub.publish(motors_state);
      published_motors_state = true;
    }
  }


  // Publish sonar information, if enabled.
  if (robot->haveSonar() && (publish_sonar || publish_sonar_pointcloud2))
  {
    sensor_msgs::PointCloud cloud;	//sonar readings.
    cloud.header.stamp = timestamp;
    // sonar sensors relative to base_link
    cloud.header.frame_id = frame_id_sonar;
  

    std::stringstream sonar_debug_info; // Log debugging info
    //sonar_debug_info << "Sonar readings: ";

    for (int i = 0; i < robot->numSonarReadings(); ++i) {
 
      int r = robot->getSonarReading(i);
      //sonar_debug_info << r << " ";

       // XXX TODO calculate x,y.  Maybe just use ARIA to help?

      // local (x,y). Appears to be from the centre of the robot, since values may
      // exceed 5000. This is good, since it means we only need 1 transform.
      // x & y seem to be swapped though, i.e. if the robot is driving north
      // x is north/south and y is east/west.
      //
      //ArPose sensor = reading->getSensorPosition();  //position of sensor.
      // sonar_debug_info << "(" << reading->getLocalX() 
      //                  << ", " << reading->getLocalY()
      //                  << ") from (" << sensor.getX() << ", " 
      //                  << sensor.getY() << ") ;; " ;
    
      //add sonar readings (robot-local coordinate frame) to cloud
      geometry_msgs::Point32 p;
      p.x = 0; //reading->getLocalX() / 1000.0;
      p.y = 0; //reading->getLocalY() / 1000.0;
      p.z = 0.0;
      cloud.points.push_back(p);
    }
    //ROS_DEBUG_STREAM(sonar_debug_info.str());
    
    // publish topic(s)

    if(publish_sonar_pointcloud2)
    {
      sensor_msgs::PointCloud2 cloud2;
      if(!sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2))
      {
        ROS_WARN("Error converting sonar point cloud message to point_cloud2 type before publishing! Not publishing this time.");
      }
      else
      {
        sonar_pointcloud2_pub.publish(cloud2);
      }
    }

    if(publish_sonar)
    {
      sonar_pub.publish(cloud);
    }
  } // end if sonar_enabled

  // TODO laser data.  any other datta

  // done publishing data
}

void ROSNode::cmdvel_cb(const geometry_msgs::TwistConstPtr& vel)
{
  // XXX TODO
}