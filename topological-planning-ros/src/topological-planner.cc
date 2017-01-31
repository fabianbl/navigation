#include "topological-planning-ros/topological-planner.h"

#include <glog/logging.h>
#include <minkindr_conversions/kindr_msg.h>
#include <multiagent-mapping-common/path-utility.h>
#include <multiagent-mapping-common/pose_types.h>
#include <pluginlib/class_list_macros.h>
#include <topological-mapping/basic-position-planner.h>
#include <topological-mapping/common.h>
#include <topological-mapping/graph-serialization.h>
#include <topological-mapping/visualization.h>

#include "topological-planning-ros/type-conversions.h"

// Register this planner as a BaseGlobalPlanner plugin.
PLUGINLIB_DECLARE_CLASS(topoplan, TopologicalPlanner, topoplan::TopologicalPlanner,
                        nav_core::BaseGlobalPlanner)

namespace topoplan {

TopologicalPlanner::TopologicalPlanner() : node_handle_("~") {}

bool TopologicalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan) {
  pose::Transformation start_tf, goal_tf;
  tf::poseMsgToKindr(start.pose, &start_tf);
  tf::poseMsgToKindr(goal.pose, &goal_tf);
  topomap::BasicPositionPlanner position_planner;
  position_planner.setStartPosition(start_tf.getPosition());
  position_planner.setGoalPosition(goal_tf.getPosition());
  topomap::Path3D path_3d;
  double path_distance_m;
  const bool is_valid_path_found =
      position_planner.planPath3D(topological_graph_, &path_3d, &path_distance_m);
  convertPath3dToPoseStamped2dPlan(path_3d, &plan);
  topomap::visualizePath3d(path_3d, "planned_path");
  return is_valid_path_found;
}

void TopologicalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  std::string file_name;
  node_handle_.getParam("topological_graph_file_name", file_name);
  CHECK(!file_name.empty());
  CHECK(common::FileExists(file_name)) << "Topological graph file does not exist.";
  CHECK(topomap::readTopologicalGraphFromFile(file_name, &topological_graph_))
      << "Could not read file.";
  topomap::visualizeGraph(topological_graph_, "topological_graph");
  LOG(INFO) << "Topological planner is initialized.";
}

}  // namespace topoplan
