#ifndef TOPOLOGICAL_PLANNING_ROS_TOPOLOGICAL_PLANNER_H_
#define TOPOLOGICAL_PLANNING_ROS_TOPOLOGICAL_PLANNER_H_

#include <string>
#include <vector>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <topological-mapping/topological-graph.h>

namespace topoplan {

// Topological planner which can be used with the ROS navigation stack.
class TopologicalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  TopologicalPlanner();
  virtual ~TopologicalPlanner() {}

  // Returns true if a valid plan for the given start and goal pose was found and false otherwise.
  virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan) override;

  // Initializes the topological graph. Both input arguments are ignored.
  virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

 private:
  topomap::TopologicalGraph topological_graph_;
  ros::NodeHandle node_handle_;
};

}  // namespace topoplan

#endif  // TOPOLOGICAL_PLANNING_ROS_TOPOLOGICAL_PLANNER_H_
