#ifndef POSE_TOOL_H
#define POSE_TOOL_H

#include <rviz/tool.h>

#include <interactive_markers/interactive_marker_server.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


namespace rviz_pose_tool
{ 

class PoseTool: public rviz::Tool
{
Q_OBJECT
public:
  PoseTool();
  ~PoseTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  visualization_msgs::Marker createLaserMarker();
  void createRobotMarker();
  void dragFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void publishInitialPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void scanCallback(const sensor_msgs::LaserScanConstPtr &laserscan);

protected:
  ros::NodeHandle nh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  visualization_msgs::Marker laserMarker_;
  laser_geometry::LaserProjection laserProjector_;
  tf::TransformListener listener_;
  tf::TransformBroadcaster broadcaster_;
  ros::Publisher marker_pub_;
  tf::StampedTransform robotCenterBaseLinkTransform_;
  tf::StampedTransform robotMarkerBaseLinkTransform_;
};

}

#endif