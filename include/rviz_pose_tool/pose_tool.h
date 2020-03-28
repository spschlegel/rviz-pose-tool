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
};

}

#endif