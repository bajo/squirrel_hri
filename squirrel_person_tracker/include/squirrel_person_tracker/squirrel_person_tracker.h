#ifndef SQUIRREL_TRACKER_H
#define SQUIRREL_TRACKER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <openni2/OpenNI.h>
#include <openni2/OniPlatform.h>
#include <string>
#include <pcl/point_types.h>
#include "squirrel_person_tracker/floor_pointer.h"
#include <geometry_msgs/PoseArray.h>
#include <squirrel_person_tracker_msgs/State.h>
#include <squirrel_person_tracker_msgs/HeadHandPoints.h>
#include <libnite2/NiTE.h>

class SquirrelTracker : public nite::UserTracker::NewFrameListener
{
protected:
  ros::NodeHandle pnh_;
public:
//  State state;
  bool ISWAVEDETECTED;
  bool SWITCHSKELTRACKON;
  bool ISWAVEUSERVISBLE;
  bool static_plane_;
  bool pub_filtered_pc_;
  double beginUnvis;
  int durationUnvis;
  std::string frame_id;
  nite::HandTracker hTracker;
  nite::HandTrackerFrameRef handFrame;
  nite::Status nstatus;
  nite::UserTracker uTracker;
  nite::UserTrackerFrameRef userFrame;
  nite::UserMap usersMap;
  nite::UserId wavingUserID;
  nite::Point3f floorPoint;
  nite::Point3f pointingHead;
  nite::Point3f pointingHand;
  nite::Plane floor_;

  openni::VideoFrameRef userDepthFrame;
  openni::Status ostatus;
  openni::Device device;
  openni::VideoStream depthSensor;
  ros::Publisher pubPC;
  ros::Publisher pubPSA;
  ros::Publisher pubPP;
  ros::Publisher pubState;
  ros::Publisher pubPHH;

  tf::TransformBroadcaster tfBraodcaster;
  tf::Transform transform;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg;
  geometry_msgs::Pose detectedPose;
  geometry_msgs::PoseArray detectedPoseArray;
  geometry_msgs::PointStamped pointingPoint;
  squirrel_person_tracker_msgs::State publishedState;
  squirrel_person_tracker_msgs::HeadHandPoints squirrelHeadHand;
  FloorPointer pDetector;

  SquirrelTracker(ros::NodeHandle& pnh_);
/////////////////////////////////////////////////////////////////////
  virtual ~SquirrelTracker();
/////////////////////////////////////////////////////////////////////
  void initOpenNI();
/////////////////////////////////////////////////////////////////////
  void initNITE();
/////////////////////////////////////////////////////////////////////
  void initUserTracker();
/////////////////////////////////////////////////////////////////////
  void shutdownOpenNI();
/////////////////////////////////////////////////////////////////////
  void shutdownNite();

/////////////////////////////////////////////////////////////////////
  void setFrameIDParameter();
/////////////////////////////////////////////////////////////////////
  void publishTransformOfPoint(const nite::UserId& userID, const nite::Point3f& point, const std::string& frame_id,
                               const std::string& child_frame_id, const ros::Time& timestamp);
/////////////////////////////////////////////////////////////////////
  void publishTransformOfJoint(const nite::UserId& userID, const nite::SkeletonJoint& joint,
                               const std::string& frame_id, const std::string& child_frame_id,
                               const ros::Time& timestamp);
/////////////////////////////////////////////////////////////////////////
  void publishPoseStampedArrayOfUsers(const nite::Point3f& point, const ros::Time& timestamp);
/////////////////////////////////////////////////////////////////////////
  void publishPointingPoint(const nite::Point3f& point, const ros::Time& timestamp);
/////////////////////////////////////////////////////////////////////////
  void publishHeadHandMsgs(const nite::Point3f& head, const nite::Point3f& hand, const ros::Time& timestamp);
/////////////////////////////////////////////////////////////////////////
  void convertDepthToFilteredPointcloud();
/////////////////////////////////////////////////////////////////////
  void publishSkeleton(const nite::UserId& userID, const nite::Skeleton& userSkeleton, const std::string& frame_id,
                       const ros::Time& timestamp);
/////////////////////////////////////////////////////////////////////
  void publishPtCld2(const ros::Time& timestamp);
/////////////////////////////////////////////////////////////////////
  bool detectWavingUser(const nite::UserId& userID);
/////////////////////////////////////////////////////////////////////
  void onNewFrame(nite::UserTracker& uTracker);
/////////////////////////////////////////////////////////////////////
  void runSquirrelTracker();
};

#endif
