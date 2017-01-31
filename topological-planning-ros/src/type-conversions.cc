#include "topological-planning-ros/type-conversions.h"

#include <glog/logging.h>
#include <multiagent-mapping-common/pose_types.h>
#include <ros/ros.h>

namespace topoplan {

void convertPath3dToPoseStamped2dPlan(const topomap::Path3D& path_3d,
                                      std::vector<geometry_msgs::PoseStamped>* pose_stamped_plan) {
  CHECK_NOTNULL(pose_stamped_plan)->clear();
  pose_stamped_plan->reserve(path_3d.size());
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  for (const pose::Position3D& position : path_3d) {
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose_stamped_plan->push_back(pose);
  }
}

}  // namespace topoplan
