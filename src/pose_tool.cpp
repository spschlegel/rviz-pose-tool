#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

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
  
}

// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
void PoseTool::activate()
{

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

} // end of namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_pose_tool::PoseTool, rviz::Tool)
