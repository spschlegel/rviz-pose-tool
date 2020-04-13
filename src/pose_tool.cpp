#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>

#include <ros/console.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

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
  ros::NodeHandle n("pose_tool");
  server = std::make_shared<interactive_markers::InteractiveMarkerServer>("pose_tool");

  // publish server init
  interactive_markers::MenuHandler menu_handler;
  ros::Publisher initialPosePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialPose", 1);
  // init publisher to initialpose
  // init tf Transfromlistener
  // init tf Transformbroadcaster
  // init laser geometry laserprojecction
  // init marker publisher
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
void PoseTool::activate()
{
  createRobotMarker();
  createLaserMarker();
  // subscribe to laser scan(s)
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

visualization_msgs::Marker PoseTool::createLaserMarker()
{
  visualization_msgs::Marker laserMarker;
  laserMarker.type = visualization_msgs::Marker::POINTS;
  laserMarker.scale.x = 0.02;
  laserMarker.scale.y = 0.02;
  laserMarker.scale.z = 0.02;
  laserMarker.color.r = 1.0;
  laserMarker.color.g = 1.0;
  laserMarker.color.b = 1.0;
  laserMarker.color.a = 1.0;
  laserMarker.header.frame_id = "robot_marker";
  laserMarker.action = visualization_msgs::Marker::ADD;

  return laserMarker;
}

void PoseTool::createRobotMarker()
{
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

  server->insert(int_marker, &dragFeedback);
  // Interactivemarkerserver.apply(interactivemarkerserver, marker.name)
  server->applyChanges();
}

void PoseTool::dragFeedback(visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // marker = interactivemarkerserver.get(feedback.marker_name)
  // broadcaster.sendTransform(trans, rot, time, roboter marker, base link)
}

} // end of namespace


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_pose_tool::PoseTool, rviz::Tool)
