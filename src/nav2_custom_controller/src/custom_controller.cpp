#include "nav2_custom_controller/custom_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/exceptions.hpp"

namespace nav2_custom_controller
{

void CustomController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // === [BLACK BOX] Basic plugin setup ===
  node_ = parent.lock();
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  clock_ = node_->get_clock();
  logger_ = node_->get_logger();

  // === [STUDENT SECTION] Declare and retrieve controller parameters ===
  // You may add any control-specific parameters here
  // Example gain parameter:
  node_->declare_parameter(name + ".kp", 1.0);
  node_->get_parameter(name + ".kp", kp_);
}

void CustomController::cleanup() {}
void CustomController::activate() {}
void CustomController::deactivate() {}

void CustomController::setPlan(const nav_msgs::msg::Path & path)
{
  // === [BLACK BOX] Called when a new global plan is received ===
  global_plan_ = path;
  goal_pose_ = path.poses.back(); // Save the final goal pose
}

// === [BLACK BOX] Transform a pose into a different frame using TF ===
bool CustomController::transformPose(
  const std::string & target_frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & /*transform_tolerance*/)
{
  if (in_pose.header.frame_id == target_frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, target_frame);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Transform error: %s", ex.what());
    return false;
  }
}

// === [STUDENT SECTION] Implement any helper functions below ===
// These functions can be used in your controller logic.

/*
// Normalize angle to [-pi, pi]
double CustomController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
*/

/*
// Euclidean distance between two poses
double CustomController::euclideanDistance(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}
*/

// === [MAIN FUNCTION TO IMPLEMENT CONTROL LOGIC] ===
geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  // === [INFO] pose: Robot's current pose in the global frame (e.g., map or odom)
  // === [INFO] global_plan_: Sequence of waypoints from planner (path to follow)

  const auto base_frame = costmap_ros_->getBaseFrameID();         // Usually "base_link"
  const auto global_frame = global_plan_.header.frame_id;         // Usually "map" or "odom"

  // === [BLACK BOX] Transform robot pose to global plan frame
  geometry_msgs::msg::PoseStamped robot_pose_in_global;
  if (!transformPose(global_frame, pose, robot_pose_in_global, rclcpp::Duration::from_seconds(0.1))) {
    throw nav2_core::PlannerException("Failed to transform robot pose to global plan frame");
  }

  // === [STUDENT SECTION] Implement your controller logic below ===
  
  //need waypoint positions

  double min_dist = std::numeric_limits<double>::max();
  size_t desired_index = 0;

  for (size_t i=0; i < global_plan_.poses.size(); ++i){
    const auto & pose = global_plan_.poses[i].pose;
    double distance_error = std::hypot(
      pose.position.x - robot_pose_in_global.pose.position.x,
      pose.position.y - robot_pose_in_global.pose.position.y
    );

    if (distance_error < min_dist){
      min_dist = distance_error;
      desired_index = i;
    }
  }

  RCLCPP_INFO(logger_, "Minimum Distance: %.3f", min_dist);
  RCLCPP_INFO(logger_, "Desired Index: %zu", desired_index);

  size_t lookahead_index = std::min(desired_index + 8, global_plan_.poses.size()-1);
  RCLCPP_INFO(logger_, "New Desired Index: %zu", lookahead_index);

  // find point on path that you want
  const auto & target_pose = global_plan_.poses[lookahead_index].pose;
  
  // now use new target on path for calculations
  double dx = target_pose.position.x - robot_pose_in_global.pose.position.x;
  double dy = target_pose.position.y - robot_pose_in_global.pose.position.y;
  double distance_error = std::hypot(dx,dy);
  
  double heading_error = std::atan2(dy,dx) - tf2::getYaw(robot_pose_in_global.pose.orientation);
  heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error)); // Normalize angle error to [-pi, pi]
  
  double dt = 0.1;
  // linear velocity control
  double kp_lin = 0.5, ki_lin = 0.1, kd_lin = 0.2;
  double previous_distance_error = 0.0, integral_distance = 0.0;

  integral_distance += distance_error*dt;
  double derivative_distance = (distance_error - previous_distance_error);
  double desired_linear_vel = kp_lin*distance_error + ki_lin*integral_distance + kd_lin*derivative_distance;
  previous_distance_error = distance_error;

  // angular velocity control
  double kp_ang = 0.8, ki_ang = 0.1, kd_ang = 0.2;
  double previous_heading_error = 0.0, integral_heading = 0.0;

  integral_heading += heading_error * dt;
  double derivative_heading = (heading_error - previous_heading_error);
  double desired_angular_vel = kp_ang * heading_error + ki_ang * integral_heading + kd_ang * derivative_heading;
  previous_heading_error = heading_error;

  // === [OUTPUT VELOCITY] Send velocity command to robot ===
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = base_frame;
  cmd_vel.twist.linear.x = std::min(desired_linear_vel, 0.5);
  cmd_vel.twist.angular.z = desired_angular_vel;
  return cmd_vel;
}

void CustomController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/) {}

}  // namespace nav2_custom_controller

// === [PLUGIN REGISTRATION] ===
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)

