/**
 * follow_child.h
 *
 * Follows a specified child based on position measurements
 *
 * @author Markus 'bajo' Bajones bajones@acin.tuwien.ac.at
 * @date Feb 2017
 */

#ifndef FOLLOW_CHILD_H
#define FOLLOW_CHILD_H

#include<math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>
#include <people_msgs/PositionMeasurementArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <squirrel_hri_msgs/FollowChildAction.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/shared_ptr.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_listener.h>
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <spencer_tracking_msgs/TrackedPersons.h>


class ChildFollowingAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *move_base_ac_;
  actionlib::SimpleActionServer<squirrel_hri_msgs::FollowChildAction> as_;  // NodeHandle instance must be
                                                                            // created before this line.
                                                                            // Otherwise strange error
                                                                 // occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  squirrel_hri_msgs::FollowChildFeedback feedback_;
  squirrel_hri_msgs::FollowChildResult result_;
  squirrel_hri_msgs::FollowChildGoal::ConstPtr goal_;
  geometry_msgs::PoseStamped point_;
  move_base_msgs::MoveBaseGoal move_base_goal_;
  tf::TransformListener listener_;
  ros::Time init_;
  geometry_msgs::Pose last_goal_;

  int id_;
  double distance_;
  double target_distance_;
  
  void LookAtChild(geometry_msgs::PoseStamped* pose, double height = 1.5);
  void publishGoalMarker(float x, float y, float z, float red, float green, float blue, const char* name);
  bool VerifyChildAtPose(geometry_msgs::PoseStamped* pose,  double &height, double margin = 0.6);

public:
  tf::StampedTransform transform;
  tf::TransformListener tfl_;
  ros::ServiceClient pan_speed_client_;
  ros::ServiceClient tilt_speed_client_;
  ros::ServiceClient pan_tilt_client_;
  ros::Publisher pan_pub_;
  ros::Publisher tilt_pub_;
  ros::Publisher vis_pub_;
  ros::Publisher pub_;
  ros::Publisher cloud_pub_;
  ros::Subscriber sub_;

  void goalCB();
  void preemptCB();
  void analysisCB(const spencer_tracking_msgs::TrackedPersons::ConstPtr &msg);
  double calculateDistance(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);

  ChildFollowingAction(std::string name);
  ~ChildFollowingAction();
};

#endif  // FOLLOW_CHILD_H
