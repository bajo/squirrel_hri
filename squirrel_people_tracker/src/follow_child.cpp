#include "squirrel_people_tracker/follow_child.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include "pcl_ros/point_cloud.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/GetMap.h>


#include <squirrel_view_controller_msgs/LookAtPosition.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef pcl::PointXYZRGB PointT;

ChildFollowingAction::~ChildFollowingAction(void)
{
  if (move_base_ac_)
    delete move_base_ac_;
}

ChildFollowingAction::ChildFollowingAction(std::string name) : as_(nh_, name, false), action_name_(name)
{
  init_ = ros::Time::now();
  id_ = 0;
  
  octomap_client_= nh_.serviceClient<std_srvs::Empty>("/squirrel_3d_mapping/reset", true);
  if (!(ros::service::waitForService(octomap_client_.getService(), ros::Duration(5.0))))
  {
    ROS_ERROR("wait for service %s failed", octomap_client_.getService().c_str());
    return;
  }

  costmap_client_= nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps", true);
  if (!(ros::service::waitForService(costmap_client_.getService(), ros::Duration(5.0))))
  {
    ROS_ERROR("wait for service %s failed", costmap_client_.getService().c_str());
    return;
  }

  pan_tilt_client_ = nh_.serviceClient<squirrel_view_controller_msgs::LookAtPosition>("/squirrel_view_controller/look_at_position", true);
  if (!(ros::service::waitForService(pan_tilt_client_.getService(), ros::Duration(5.0))))
  {
    ROS_ERROR("wait for service %s failed", pan_tilt_client_.getService().c_str());
    return;
  }

  static_map_client_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map", true);
  if (!(ros::service::waitForService(static_map_client_.getService(), ros::Duration(5.0))))
  {
    ROS_ERROR("wait for service %s failed", static_map_client_.getService().c_str());
    return;
  }

  move_base_ac_ = new MoveBaseClient("move_base", true);
  if (!move_base_ac_->waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR("Waiting for the move_base action server to come up");
    return;
  }

  distance_ = 0.8;
  // register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&ChildFollowingAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ChildFollowingAction::preemptCB, this));

  costmap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1, &ChildFollowingAction::processCostmapCB, this);
  as_.start();

  // publishers
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>("published_topic", 1);
  vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  octomap_pub_ = nh_.advertise<std_msgs::Bool>( "squirrel_3d_mapping/update", 0 );
  cloud_pub_ = nh_.advertise<pcl::PointCloud<PointT> > ("filtered_cloud", 5, true);
  ROS_INFO("Ready to accept goals...");
}

void ChildFollowingAction::printGridMap()
{
  std::cout << "Map frame: " << map_.getFrameId() << std::endl;
  std::cout << "Map resolution: " << map_.getResolution() << " size x: " << map_.getSize()(0) << " y: "
            << map_.getSize()(1) << std::endl;
  std::cout << "Map dimensions in meters. x: " << map_.getLength().x() << " y: " << map_.getLength().y()
            << std::endl;
  grid_map::Matrix& data = map_["static"];
  grid_map::Position position;
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    map_.getPosition(index, position);
    if (data(index(0), index(1)) > 90)
        std::cout << "The value at index " << index.transpose() << " is " << data(index(0), index(1)) 
            << "at position: " << position << std::endl;
        
  }
}

bool ChildFollowingAction::verifyDetection(const geometry_msgs::PoseStamped pose)
{
  geometry_msgs::PoseStamped map_pose;
  try{
    ros::Time now = ros::Time(0);
    tfl_.waitForTransform(pose.header.frame_id, "map",
                            now, ros::Duration(3.0));
    tfl_.transformPose("map", pose, map_pose);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }

  std::cout << map_pose.header.frame_id << std::endl;
  grid_map::Position position(map_pose.pose.position.x, map_pose.pose.position.y);
  if (map_.atPosition("static", position) < 40)
      return true;
  return false;
}

void ChildFollowingAction::processCostmapCB(const nav_msgs::OccupancyGridConstPtr& msg)
{
  std::cout << "map: resolution: " << msg->info.resolution << std::endl;
  grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "static", map_);
  //printGridMap();
  costmap_sub_.shutdown();

}


void ChildFollowingAction::goalCB()
{
  goal_ = as_.acceptNewGoal();
  /*
   for (size_t i=0; i < goal_->target_locations.size(); ++i)
  {
    ROS_INFO("publish target location markers.");
    publishGoalMarker(goal_->target_locations[i].x, goal_->target_locations[i].y, 0.0, 1.0, 0.0, 0.0, "child_target_locations");
    id_ += 1;
  }
  */
}

void ChildFollowingAction::preemptCB()
{
  std_msgs::Bool octomap_update;
  octomap_update.data = true;
  octomap_pub_.publish(octomap_update);
  ROS_INFO("%s: Preempted", action_name_.c_str());
  // set the action state to preempted
  as_.setPreempted();
}

double ChildFollowingAction::calculateDistanceFromRobot(geometry_msgs::PoseStamped pose)
{
  geometry_msgs::PoseStamped laser_pose, child_pose;
  double distance = 1000.0;
  double x;
  double y;
  ros::Time now = ros::Time::now();
  laser_pose.header.stamp = ros::Time(0);
  laser_pose.header.frame_id = "hokuyo_link";
  laser_pose.pose.position.x = 0.0;
  laser_pose.pose.position.y = 0.0;
  laser_pose.pose.orientation =  tf::createQuaternionMsgFromYaw(0.0);

  try{
      tfl_.waitForTransform(pose.header.frame_id, laser_pose.header.frame_id, now, ros::Duration(1.0));
      tfl_.transformPose(pose.header.frame_id, pose, child_pose);
  } catch (tf::TransformException ex) {
      ROS_WARN("Failed to retrieve most recent transfrom.");
      return 1000.0;
  }

  x = laser_pose.pose.position.x - child_pose.pose.position.x;
  y = laser_pose.pose.position.y - child_pose.pose.position.y;

  distance = pow(x, 2) + pow(y, 2);
  distance = sqrt(distance);
  return distance;
}

void ChildFollowingAction::analysisCB(const spencer_tracking_msgs::TrackedPersons::ConstPtr &msg)
{
  // make sure that the action hasn't been canceled
  if (!as_.isActive())
  {
    ROS_DEBUG("Action server %s is no longer active. Exiting.", action_name_.c_str());
    return;
  }
  ROS_DEBUG("Data received and action started.");
  if (msg->tracks.size() == 0)
  {
    ROS_DEBUG("No people in message"); 
    return;
  }
  std_msgs::Bool octomap_update;
  octomap_update.data = false;
  octomap_pub_.publish(octomap_update);
  std_srvs::Empty empty;
  octomap_client_.call(empty);
  costmap_client_.call(empty);

  geometry_msgs::PoseStamped robot_pose, child_pose, tmp_pose, out_pose, detection_pose;
  move_base_msgs::MoveBaseGoal move_base_goal_;
  double min_distance = 1000.0;
  int index = 0;
  double height = 0.0;
  double time_diff = (ros::Time::now() - init_).toSec();
  bool child_present = false;

  ROS_DEBUG("time diff: %f", time_diff);
  if (time_diff < 1.0)
  {
    return;
  }

  // calculate distance to select the closest personCB
  detection_pose.header.stamp = ros::Time(0);
  detection_pose.header.frame_id = msg->header.frame_id;

  child_pose.header.stamp = ros::Time(0);
  //child_pose.header.frame_id = "map";
  double distance;

  for (size_t i = 0; i < msg->tracks.size(); ++i)
  {
    detection_pose.pose = msg->tracks[i].pose.pose;
    if (!verifyDetection(detection_pose))
        continue;

    distance = calculateDistanceFromRobot(detection_pose);
    //tmp_pose.pose.position.z = 1.3;
    //LookAtChild(&tmp_pose);

    ROS_DEBUG("Current distance is: %lf", distance);
    if (distance < min_distance)
    {
      //LookAtChild(&tmp_pose, height);
      index = i;
      min_distance = distance;
    }
  }
  detection_pose.pose = msg->tracks[index].pose.pose;
  ROS_DEBUG("Closest distance is: %lf", min_distance);

  try{
    ros::Time now = ros::Time(0);
    tfl_.waitForTransform(detection_pose.header.frame_id, "map",
                            now, ros::Duration(3.0));
    tfl_.transformPose("map", detection_pose, child_pose);
    tfl_.waitForTransform(detection_pose.header.frame_id, "hokuyo_link",
                          now, ros::Duration(3.0));
    tfl_.transformPose("hokuyo_link", detection_pose, tmp_pose);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  ROS_DEBUG("Person detected at (x, y): (%f, %f) %s", child_pose.pose.position.x, child_pose.pose.position.y, child_pose.header.frame_id.c_str());
  ROS_DEBUG("Person detected at (x, y): (%f, %f) %s", tmp_pose.pose.position.x, tmp_pose.pose.position.y, tmp_pose.header.frame_id.c_str());

  // check if the child is in one of the target areas
  for (size_t i=0; i < goal_->target_locations.size(); ++i)
  {
    if ((fabs(child_pose.pose.position.x - goal_->target_locations[i].x) < 0.5) &&
        (fabs(child_pose.pose.position.y - goal_->target_locations[i].y) < 0.5))
    {
      octomap_update.data = true;
      octomap_pub_.publish(octomap_update);
      // make sure we stop now
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      result_.final_location = child_pose;
      as_.setSucceeded(result_);
    }
  }
  
  // calculate a point between the child and the robot
  double alpha = atan2(tmp_pose.pose.position.y, tmp_pose.pose.position.x);
  double k = sqrt(tmp_pose.pose.position.x * tmp_pose.pose.position.x + tmp_pose.pose.position.y * tmp_pose.pose.position.y);
  ROS_DEBUG("k: %f, alpha: %f", k, alpha);
  ROS_DEBUG("sin(alpha): %f, cos(alpha): %f", sin(alpha), cos(alpha));
  
  tmp_pose.header.stamp = ros::Time(0);
  tmp_pose.header.frame_id = "hokuyo_link";
  tmp_pose.pose.position.x = (k - distance_)*cos(alpha);
  tmp_pose.pose.position.y = (k - distance_)*sin(alpha);
  tmp_pose.pose.orientation = tf::createQuaternionMsgFromYaw(alpha);
  
  pub_.publish(tmp_pose);
  try{
    ros::Time now = ros::Time(0);
    tfl_.waitForTransform("hokuyo_link", "map",
                            now, ros::Duration(3.0));
    tfl_.transformPose("map", tmp_pose, out_pose);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
  if (fabs(last_goal_.position.x - out_pose.pose.position.x) < 0.10 &&
      fabs(last_goal_.position.y - out_pose.pose.position.y) < 0.10 &&
      fabs(tf::getYaw(last_goal_.orientation) - tf::getYaw(out_pose.pose.orientation) < 0.26)) //~15 degree
  {
  ROS_DEBUG("Last goal was (x, y): (%f, %f) map", last_goal_.position.x, last_goal_.position.y);
  ROS_DEBUG("Current nav goal would be (x, y): (%f, %f) map", out_pose.pose.position.x, out_pose.pose.position.y);
    ROS_INFO("current goal and last goal are close to each other. Do not send new goal");
  publishGoalMarker(out_pose.pose.position.x, out_pose.pose.position.y, out_pose.pose.position.z, 0.0, 1.0, 0.0, "child_goal");
    return;
  }

  publishGoalMarker(out_pose.pose.position.x, out_pose.pose.position.y, out_pose.pose.position.z, 0.0, 1.0, 0.0, "child_goal");
  ROS_DEBUG("Setting nav goal to (x, y): (%f, %f) hokuyo_link", tmp_pose.pose.position.x, tmp_pose.pose.position.y);
  ROS_INFO("Setting nav goal to (x, y): (%f, %f) map", out_pose.pose.position.x, out_pose.pose.position.y);

  last_goal_.position = out_pose.pose.position;
  last_goal_.orientation = out_pose.pose.orientation;

  // set move_base goal
  move_base_goal_.target_pose.header.frame_id = out_pose.header.frame_id;
  move_base_goal_.target_pose.header.stamp = out_pose.header.stamp;
  move_base_goal_.target_pose.pose.position.x = out_pose.pose.position.x;
  move_base_goal_.target_pose.pose.position.y = out_pose.pose.position.y;
  move_base_goal_.target_pose.pose.orientation = out_pose.pose.orientation;

  ROS_INFO("Sending goal to move_base");
  move_base_ac_->sendGoal(move_base_goal_);
  //LookAtChild(&child_pose);

  init_ = ros::Time::now();
  ros::Duration(0.25).sleep();
}

void ChildFollowingAction::publishGoalMarker(float x, float y, float z, float red, float green, float blue, const char* name)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time(0);
  marker.ns = name;
  marker.id = id_;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue; 
  vis_pub_.publish(marker);
  ros::Duration(0.01).sleep();
}

bool ChildFollowingAction::VerifyChildAtPose(geometry_msgs::PoseStamped* pose, double &height, double margin)
{
  sensor_msgs::PointCloud2ConstPtr sceneConst;
  sensor_msgs::PointCloud2 scene;
  geometry_msgs::PointStamped point, point_max;
  geometry_msgs::PoseStamped out_pose;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr outputCloud(new pcl::PointCloud<PointT>);

  // get data from depth camera
  sceneConst =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", nh_, ros::Duration(20));

  // cut away area that is not close to the child candidate's location
  if (sceneConst != NULL)
  {
    scene = *sceneConst;
    pcl::fromROSMsg(scene, *cloud);
    ROS_INFO("cloud frame is %s", scene.header.frame_id.c_str());

    try
    {
      ros::Time now = ros::Time(0);
      listener_.waitForTransform(scene.header.frame_id, pose->header.frame_id, now, ros::Duration(3.0));
      listener_.transformPose(scene.header.frame_id, now, *pose, pose->header.frame_id, out_pose);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = out_pose.header.frame_id;
    marker.header.stamp = ros::Time(0);
    marker.ns = "cutoff";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = out_pose.pose.position.x;
    marker.pose.position.y = out_pose.pose.position.y;
    marker.pose.position.z = out_pose.pose.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    vis_pub_.publish(marker);
    ros::Duration(0.01).sleep();

    try
    {
	// Create the filtering object
        pcl::io::savePCDFileASCII("/tmp/cloud.pcd", *cloud);
    }
    catch (pcl::IOException ex)
    {
	ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
        return false;
    }

    pcl::PassThrough<PointT> pass;
    pass.setKeepOrganized(true);
    pass.setFilterFieldName("z");
    ROS_INFO("cutting at %s frame : %f", pose->header.frame_id.c_str(), pose->pose.position.x);
    ROS_INFO("cutting Z at %s frame : %f",out_pose.header.frame_id.c_str(), out_pose.pose.position.z);
    pass.setFilterLimits(out_pose.pose.position.z - margin, out_pose.pose.position.z + margin);
    pass.setInputCloud(cloud);
    pass.filter(*cloud);
    pass.setFilterFieldName("x");
    ROS_INFO("cutting X at %s frame : %f",out_pose.header.frame_id.c_str(), out_pose.pose.position.x);
    pass.setFilterLimits(out_pose.pose.position.x - margin, out_pose.pose.position.x + margin);
    pass.setInputCloud(cloud);
    pass.filter(*cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);
    cloud_pub_.publish(outputCloud);
    PointT min_p, max_p;
    pcl::getMinMax3D(*outputCloud, min_p, max_p);

    try
    {
	// Create the filtering object
        pcl::io::savePCDFileASCII("/tmp/filtered_cloud.pcd", *outputCloud);
    }
    catch (pcl::IOException ex)
    {
	ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
        return false;
    }
    ROS_INFO("min y: %f, max y: %f", min_p.y, max_p.y);

    point.header.frame_id = out_pose.header.frame_id;
    point.header.stamp = out_pose.header.stamp;
    point.point.x = out_pose.pose.position.x;
    point.point.y = min_p.y;
    point.point.z = out_pose.pose.position.z;

    ROS_INFO("Highest point is at: x: %f, y: %f, z: %f in frame: %s", point.point.x, point.point.y, point.point.z, point.header.frame_id.c_str());

    marker.header.frame_id = scene.header.frame_id;
    marker.header.stamp = ros::Time(0);
    marker.ns = "cutoff";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.point.x;
    marker.pose.position.y = point.point.y;
    marker.pose.position.z = point.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    vis_pub_.publish(marker);
    ros::Duration(0.01).sleep();

    try
    {
      ros::Time now = ros::Time(0);
      listener_.waitForTransform("/map", point.header.frame_id, now, ros::Duration(3.0));
      listener_.transformPoint("/map", now, point, point.header.frame_id, point_max);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }

    ROS_INFO("Highest point: %f", point_max.point.z);
    // plausibility check
    if ((point_max.point.z < 1.0) || (point_max.point.z > 2.0))
    {
      // height check failed
      ROS_INFO("Highest point is lower than 1.0 or above 2.0 meters. Unlikely to be a standing child.");
      return false;
    }
    height = point_max.point.z;
    ROS_INFO("Height check success");
    return true;
  }
  return false;
}

void ChildFollowingAction::LookAtChild(geometry_msgs::PoseStamped* pose, double height)
{
  squirrel_view_controller_msgs::LookAtPosition srv;
  // set pan / tilt goal
  srv.request.target.header.frame_id = pose->header.frame_id;
  srv.request.target.header.stamp = pose->header.stamp;
  srv.request.target.pose.position.x = pose->pose.position.x;
  srv.request.target.pose.position.y = pose->pose.position.y;
  srv.request.target.pose.position.z = height;
  srv.request.target.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  if (pan_tilt_client_.call(srv))
  {
    ROS_DEBUG("%s: Reached position", pan_tilt_client_.getService().c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service %s", pan_tilt_client_.getService().c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "squirrel_child_follower");
  ChildFollowingAction follow_child(ros::this_node::getName());
  ros::spin();
  return 0;
}
