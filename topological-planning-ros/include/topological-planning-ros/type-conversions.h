#ifndef TOPOLOGICAL_PLANNING_ROS_TYPE_CONVERSIONS_H_
#define TOPOLOGICAL_PLANNING_ROS_TYPE_CONVERSIONS_H_

#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <topological-mapping/common.h>

namespace topoplan {

// Converts a 3D path to a vector of 2d PoseStamped messages. The rotation of each PoseStamped
// message is set to the unit quaternion as this does not matter for the global plan.
// TODO(fabianbl): Check if this is true that the orientation does not need to be set.
void convertPath3dToPoseStamped2dPlan(const topomap::Path3D& path_3d,
                                      std::vector<geometry_msgs::PoseStamped>* pose_stamped_plan);

}  // namespace topoplan

#endif  // TOPOLOGICAL_PLANNING_ROS_TYPE_CONVERSIONS_H_
