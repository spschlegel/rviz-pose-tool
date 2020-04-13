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
  virtual void dragFeedback(visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

protected:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
};

}

#endif