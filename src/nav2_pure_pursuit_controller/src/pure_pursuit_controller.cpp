/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *  Contributor: Pham Cong Trang <phamcongtranghd@gmail.com>
 *  Contributor: Mitchell Sayer <mitchell4408@gmail.com>
 */

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_pure_pursuit_controller/pure_pursuit_controller.hpp"
#include "nav2_util/geometry_utils.hpp"


using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace nav2_pure_pursuit_controller
{

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(
      0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist",
    rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
      1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
      0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".target_xy_tolerance", rclcpp::ParameterValue(
      0.2));
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".target_xy_tolerance", target_xy_tolerance_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void PurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type pure_pursuit_controller::PurePursuitController",
    plugin_name_.c_str());
  global_pub_.reset();
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
  global_pub_->on_activate();
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
  global_pub_->on_deactivate();
}

void PurePursuitController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  (void) speed_limit;
  (void) percentage;
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;
  node_.lock()->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_.lock()->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node_.lock()->get_parameter(plugin_name_ + ".target_xy_tolerance", target_xy_tolerance_);
  auto transformed_plan = transformGlobalPlan(pose);

  //防止隔墙的bug
  if (!global_plan_.poses.empty()) 
  {
    double start_distance = hypot(
      global_plan_.poses[0].pose.position.x-pose.pose.position.x,
      global_plan_.poses[0].pose.position.y-pose.pose.position.y);
    if(start_distance<0.07){
      stick_cout++;
      if (stick_cout>180){
        stick_cout=181;
      }
      if (stick_cout==90){
        stick_cout=181;
      }
    }
    else{
      stick_cout-=4;
      if (stick_cout<0){
        stick_cout=0;
      }
    }
    if(stick_cout>90)
      {
        // RCLCPP_ERROR(
        //   rclcpp::get_logger("tf_help"),
        //   "卡住自救%f",start_distance
        // );
        lookahead_dist_=0.1;
        target_xy_tolerance_=0.0;
      }
  }
  // Find the first pose which is at a distance greater than the specified lookahed distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_;
    });

  double linear_vel_x,linear_vel_y;
  // If the last pose is still within lookahed distance, take the last pose
  // bool near=false;
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
    // near=true;
  }
  auto goal_pose = goal_pose_it->pose;
  if (goal_pose.position.x > target_xy_tolerance_) {
    linear_vel_x = desired_linear_vel_;
  } else if(goal_pose.position.x < -target_xy_tolerance_){
    linear_vel_x = -desired_linear_vel_;
  }else {
    linear_vel_x = 0.0;
    // RCLCPP_WARN(logger_, "直行:x=%f y=%f",goal_pose.position.x,goal_pose.position.y);
  }
  if (goal_pose.position.y > target_xy_tolerance_) {
    linear_vel_y = desired_linear_vel_;
  } else if(goal_pose.position.y < -target_xy_tolerance_){
    linear_vel_y = -desired_linear_vel_;
  }else {
    linear_vel_y = 0.0;
      // RCLCPP_WARN(logger_, "直行:x=%f y=%f",goal_pose.position.x,goal_pose.position.y);
  }

  // double distance = hypot(
  //   goal_pose.position.x,
  //   goal_pose.position.y
  // );

  if (distance_to_destination<1) {//邻近距离减速
    // RCLCPP_ERROR(
    //     rclcpp::get_logger("tf_help"),
    //     "快到目的了%f",distance_to_destination
    //   );
    linear_vel_x = linear_vel_x*distance_to_destination;
    linear_vel_y = linear_vel_y*distance_to_destination;
  }
  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel_x;
  cmd_vel.twist.linear.y = linear_vel_y;
  // RCLCPP_WARN(logger_, "linear_vel_x=%f,linear_vel_y=%f",linear_vel_x,linear_vel_y);

  return cmd_vel;
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;
}

nav_msgs::msg::Path
PurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Original mplementation taken fron nav2_dwb_controller

  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }
  distance_to_destination = calculateDistanceToDestination(pose, global_plan_);
  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin =
    min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // From the closest point, look for the first point that's further then dist_threshold from the
  // robot. These points are definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
    });

  // Helper function for the transform below. Transforms a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
        tf_, costmap_ros_->getBaseFrameID(),
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}
double PurePursuitController::calculateDistanceToDestination(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & global_plan)
{
  double min_distance = std::numeric_limits<double>::max();
  double distance_to_destination = 0.0;

  // 遍历全局路径上的每个点
  for (const auto & point : global_plan.poses)
  {
    // 计算机器人当前位置到该点的距离
    double distance = euclidean_distance(pose, point);

    // 找到距离最小的点
    if (distance < min_distance)
    {
      min_distance = distance;
    }
  }

  // 最近点到全局路径终点的距离，即为机器人到目的地的距离
  distance_to_destination = euclidean_distance(global_plan.poses.back(), global_plan.poses.front());
  // RCLCPP_ERROR(
  //       rclcpp::get_logger("tf_help"),
  //       "distance_to_destination%f",distance_to_destination
  //     );
  return distance_to_destination;
}
bool PurePursuitController::transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance
) const
{
  // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException & ex) {
    auto transform = tf->lookupTransform(
      frame,
      in_pose.header.frame_id,
      tf2::TimePointZero
    );
    if (
      (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Transform data too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(),
        frame.c_str()
      );
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Data time: %ds %uns, Transform time: %ds %uns",
        in_pose.header.stamp.sec,
        in_pose.header.stamp.nanosec,
        transform.header.stamp.sec,
        transform.header.stamp.nanosec
      );
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("tf_help"),
      "Exception in transformPose: %s",
      ex.what()
    );
    return false;
  }
  return false;
}

}  // namespace nav2_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)