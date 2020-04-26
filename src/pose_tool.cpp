#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/console.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <rviz/display_context.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include "rviz_pose_tool/pose_tool.h"



namespace rviz_pose_tool
{ 

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
PoseTool::PoseTool()
{ 
  shortcut_key_ = 'p';
}

// The destructor for a Tool subclass is only called when the tool
// is removed from the toolbar with the "-" button.
PoseTool::~PoseTool()
{

}

// Should be called only once per instantiation.
// onInitialize() is called during initial instantiation of the tool object.
// At this point the tool has not been activated yet.
void PoseTool::onInitialize()
{
  nh_ = ros::NodeHandle("pose_tool");
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("pose_tool");

  ROS_WARN("Setting up marker menu handler");
  interactive_markers::MenuHandler menu_handler;
  menu_handler.insert("Done", boost::bind(&PoseTool::publishInitialPose, this, _1));

  ROS_WARN("Setting up publisher to initialpose");
  ros::Publisher initialPosePub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialPose", 1);

  ROS_WARN("Initializing transform between robot_center and base_link");
  try
  {
    listener_.waitForTransform("base_scan", "base_link", ros::Time(0), ros::Duration(5));
    listener_.lookupTransform("base_scan", "base_link", ros::Time(0), robotCenterBaseLinkTransform_);
    
  }
  catch(const tf::LookupException)
  {
    ROS_ERROR("Transform from robot_center to base_link not available!");
  }
  
  ROS_WARN("Broadcasting inital transform for robot_marker to base_link");
  try
  {
    robotMarkerBaseLinkTransform_.child_frame_id_ = "robot_marker";
    robotMarkerBaseLinkTransform_.frame_id_ = "base_link";
    robotMarkerBaseLinkTransform_.stamp_ = robotCenterBaseLinkTransform_.stamp_;
    robotMarkerBaseLinkTransform_.setOrigin(robotCenterBaseLinkTransform_.getOrigin());
    robotMarkerBaseLinkTransform_.setRotation(robotCenterBaseLinkTransform_.getRotation());
    broadcaster_.sendTransform(robotMarkerBaseLinkTransform_);
  }
  catch(const std::exception& e)
  {
   ROS_ERROR("Could not send initial transform for robot_marker");
  }
    
  ROS_WARN("Setting up publisher to laser_marker");
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("laser_marker", 1);
}

// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
void PoseTool::activate()
{
  createRobotMarker();
  createLaserMarker();
  ROS_WARN("Setting up laser scan subscriber");
  ros::Subscriber sub = nh_.subscribe("laserscan", 1, &PoseTool::scanCallback, this);
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
void PoseTool::deactivate()
{

}

int PoseTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                     ground_plane,
                                     event.x, event.y, intersection );
  
  return Render;

}

void PoseTool::scanCallback(const sensor_msgs::LaserScanConstPtr& laserscan)
{
  laserMarker_.points.clear();

  sensor_msgs::LaserScan laserScan = *laserscan;
  
  sensor_msgs::PointCloud2 pointCloudMsg;
  laserProjector_.projectLaser(laserScan, pointCloudMsg);
  
  for (sensor_msgs::PointCloud2Iterator<float> it(pointCloudMsg, "scanPoint"); it != it.end(); ++it) {
    geometry_msgs::PointStamped pointStamped;
    geometry_msgs::Point point;
    point.x = it[0];
    point.x = it[1];
    point.x = it[2];
    pointStamped.point = point;
    pointStamped.header.frame_id = laserScan.header.frame_id;
    geometry_msgs::PointStamped transformedStampedPoint;
    listener_.transformPoint("base_link", pointStamped, transformedStampedPoint);
    geometry_msgs::Point transformedPoint;
    transformedPoint.x = transformedStampedPoint.point.x - robotCenterBaseLinkTransform_.getOrigin().x();
    transformedPoint.y = transformedStampedPoint.point.y - robotCenterBaseLinkTransform_.getOrigin().y();
    transformedPoint.z = transformedStampedPoint.point.z; 
    laserMarker_.points.push_back(transformedPoint);
  }
  try
  {
    marker_pub_.publish(laserMarker_);
  }
  catch(const std::exception& e){}
}

visualization_msgs::Marker PoseTool::createLaserMarker()
{
  ROS_WARN("Creating laser marker");
  laserMarker_.type = visualization_msgs::Marker::POINTS;
  laserMarker_.scale.x = 0.02;
  laserMarker_.scale.y = 0.02;
  laserMarker_.scale.z = 0.02;
  laserMarker_.color.r = 1.0;
  laserMarker_.color.g = 1.0;
  laserMarker_.color.b = 1.0;
  laserMarker_.color.a = 1.0;
  laserMarker_.header.frame_id = "robot_marker";
  laserMarker_.action = visualization_msgs::Marker::ADD;

  return laserMarker_;
}

void PoseTool::createRobotMarker()
{
  ROS_WARN("Creating robot marker");
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.name = "robot_marker";
  int_marker.description = "Roboter marker";
  int_marker.scale = 1;
  geometry_msgs::Pose pose;
  int_marker.pose = pose;

  // drag control
  visualization_msgs::InteractiveMarkerControl drag_control;
  drag_control.name = "drag";
  drag_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  int_marker.controls.push_back(drag_control);

  visualization_msgs::Marker dragMarker;
  dragMarker.type = visualization_msgs::Marker::CUBE;
  dragMarker.scale.x = 0.02;
  dragMarker.scale.y = 0.02;
  dragMarker.scale.z = 0.02;
  dragMarker.color.r = 1.0;
  dragMarker.color.g = 1.0;
  dragMarker.color.b = 1.0;
  dragMarker.color.a = 1.0;
  dragMarker.header.frame_id = "robot_marker";
  dragMarker.action = visualization_msgs::Marker::ADD;
  dragMarker.header.frame_id="robot_marker";
  dragMarker.action=visualization_msgs::Marker::ADD;

  drag_control.markers.push_back(dragMarker);

  // rot control
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.name = "rotate";
  rotate_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

  visualization_msgs::Marker rotate_marker1;
  rotate_marker1.type = visualization_msgs::Marker::CUBE;
  rotate_marker1.scale.x = 0.02;
  rotate_marker1.scale.y = 0.02;
  rotate_marker1.scale.z = 0.02;
  rotate_marker1.color.r = 1.0;
  rotate_marker1.color.g = 1.0;
  rotate_marker1.color.b = 1.0;
  rotate_marker1.color.a = 1.0;
  rotate_marker1.header.frame_id = "robot_marker";
  rotate_marker1.action = visualization_msgs::Marker::ADD;
  rotate_marker1.header.frame_id="robot_marker";
  rotate_marker1.action=visualization_msgs::Marker::ADD;

  rotate_control.markers.push_back(rotate_marker1);

  visualization_msgs::Marker rotate_marker2;
  rotate_marker2.type = visualization_msgs::Marker::ARROW;
  rotate_marker2.scale.x = 0.02;
  rotate_marker2.scale.y = 0.02;
  rotate_marker2.scale.z = 0.02;
  rotate_marker2.color.r = 1.0;
  rotate_marker2.color.g = 1.0;
  rotate_marker2.color.b = 1.0;
  rotate_marker2.color.a = 1.0;
  rotate_marker2.header.frame_id = "robot_marker";
  rotate_marker2.action = visualization_msgs::Marker::ADD;
  rotate_marker2.header.frame_id="robot_marker";
  rotate_marker2.action=visualization_msgs::Marker::ADD;

  rotate_control.markers.push_back(rotate_marker2);

  int_marker.controls.push_back(rotate_control);

  server_->insert(int_marker, boost::bind(&PoseTool::dragFeedback, this, _1));
  // Interactivemarkerserver_.apply(interactivemarkerserver_, marker.name)
  server_->applyChanges();
}

void PoseTool::dragFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // marker = interactivemarkerserver_.get(feedback.marker_name)
  tf::Vector3 trans;
  tf::Quaternion rot;
  tf::StampedTransform robotMarkerBaseLinkTransform_;
  robotMarkerBaseLinkTransform_.child_frame_id_ = "robot_marker";
  robotMarkerBaseLinkTransform_.frame_id_ = "base_link";
  robotMarkerBaseLinkTransform_.stamp_ = ros::Time(0);
  robotMarkerBaseLinkTransform_.setOrigin(trans);
  robotMarkerBaseLinkTransform_.setRotation(rot);
  broadcaster_.sendTransform(robotMarkerBaseLinkTransform_);
}

void PoseTool::publishInitialPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  server_->applyChanges();
}

} // end of namespace


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_pose_tool::PoseTool, rviz::Tool)
