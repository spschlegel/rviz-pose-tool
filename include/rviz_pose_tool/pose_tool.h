#ifndef POSE_TOOL_H
#define POSE_TOOL_H

#include <rviz/tool.h>

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

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  visualization_msgs::Marker createLaserMarker();
  virtual void createRobotMarker();
  virtual void dragFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  virtual void publishInitialPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  virtual void scanCallback(const sensor_msgs::LaserScanConstPtr &laserscan);

protected:
  ros::NodeHandle nh;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  visualization_msgs::Marker laserMarker;
  laser_geometry::LaserProjection laserProjector;
  tf::TransformListener listener_;
  ros::Publisher marker_pub;
  tf::StampedTransform robotCenterBaseLinkTransform_;
};

}

#endif