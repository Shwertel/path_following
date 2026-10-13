#include "egrobots_lane_controller/lane_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace egrobots_lane_controller
{

static double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

void LaneController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  using nav2_util::declare_parameter_if_not_declared;
  auto p = [&](const std::string & key) { return plugin_name_ + "." + key; };

  declare_parameter_if_not_declared(node, p("desired_linear_vel"), rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, p("max_angular_vel"), rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(node, p("min_pivot_speed"), rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, p("align_tolerance_deg"), rclcpp::ParameterValue(4.0));
  declare_parameter_if_not_declared(node, p("heading_kp"), rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(node, p("cross_track_kp"), rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, p("max_correction_deg"), rclcpp::ParameterValue(50.0));
  declare_parameter_if_not_declared(node, p("cross_track_deadband"), rclcpp::ParameterValue(0.03));
  declare_parameter_if_not_declared(node, p("lane_offset"), rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, p("lane_timeout"), rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, p("lookahead_dist"), rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, p("lane_topic"), rclcpp::ParameterValue(std::string("/lane_centreline")));

  double align_deg, correction_deg;
  node->get_parameter(p("desired_linear_vel"), desired_linear_vel_);
  node->get_parameter(p("max_angular_vel"), max_angular_vel_);
  node->get_parameter(p("min_pivot_speed"), min_pivot_speed_);
  node->get_parameter(p("align_tolerance_deg"), align_deg);
  node->get_parameter(p("heading_kp"), heading_kp_);
  node->get_parameter(p("cross_track_kp"), cross_track_kp_);
  node->get_parameter(p("max_correction_deg"), correction_deg);
  node->get_parameter(p("cross_track_deadband"), cross_track_deadband_);
  node->get_parameter(p("lane_offset"), lane_offset_);
  node->get_parameter(p("lane_timeout"), lane_timeout_);
  node->get_parameter(p("lookahead_dist"), lookahead_);
  node->get_parameter(p("lane_topic"), lane_topic_);
  align_tolerance_ = align_deg * M_PI / 180.0;
  max_correction_ = correction_deg * M_PI / 180.0;

  lane_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    lane_topic_, rclcpp::QoS(10),
    std::bind(&LaneController::laneCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "%s: steering by %s, falling back to the plan after %.1f s without it",
    plugin_name_.c_str(), lane_topic_.c_str(), lane_timeout_);
}

void LaneController::cleanup() { lane_sub_.reset(); }
void LaneController::activate() {}
void LaneController::deactivate() {}

void LaneController::setPlan(const nav_msgs::msg::Path & path) { global_plan_ = path; }

void LaneController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    speed_limit_ = 1.0;
  } else {
    speed_limit_ = percentage ? speed_limit / 100.0 : speed_limit / desired_linear_vel_;
  }
  speed_limit_ = std::clamp(speed_limit_, 0.0, 1.0);
}

void LaneController::laneCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  lane_ = *msg;
  have_lane_ = true;
}

bool LaneController::measuredLane(double & x, double & y, double & yaw)
{
  if (!have_lane_) {
    return false;
  }
  const double age =
    (clock_->now() - rclcpp::Time(lane_.header.stamp, clock_->get_clock_type())).seconds();
  if (age < 0.0 || age > lane_timeout_) {
    return false;
  }

  geometry_msgs::msg::PoseStamped in_costmap_frame;
  try {
    tf_->transform(lane_, in_costmap_frame, costmap_ros_->getGlobalFrameID(),
                   tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Lane transform failed: %s", e.what());
    return false;
  }

  x = in_costmap_frame.pose.position.x;
  y = in_costmap_frame.pose.position.y;
  yaw = tf2::getYaw(in_costmap_frame.pose.orientation);
  return true;
}

bool LaneController::headingFromPlan(
  const geometry_msgs::msg::PoseStamped & pose, double & heading)
{
  if (global_plan_.poses.empty()) {
    return false;
  }

  // The plan is planned in the global costmap's frame (map); the pose handed
  // to a controller is in the LOCAL costmap's frame (odom). Comparing them
  // directly silently offsets the carrot by the whole map->odom correction -
  // measured, that walked the rover from 4 cm off the lane centre to 32 cm,
  // far enough for the collision monitor to freeze it against a crop row.
  geometry_msgs::msg::TransformStamped plan_to_costmap;
  try {
    plan_to_costmap = tf_->lookupTransform(
      costmap_ros_->getGlobalFrameID(), global_plan_.header.frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Plan transform failed: %s", e.what());
    return false;
  }

  auto to_costmap_frame = [&](const geometry_msgs::msg::PoseStamped & in) {
    geometry_msgs::msg::PoseStamped out;
    tf2::doTransform(in, out, plan_to_costmap);
    return out;
  };

  // The first pose on the plan at least lookahead_ away: the carrot.
  for (const auto & p : global_plan_.poses) {
    const auto q = to_costmap_frame(p);
    const double dx = q.pose.position.x - pose.pose.position.x;
    const double dy = q.pose.position.y - pose.pose.position.y;
    if (std::hypot(dx, dy) >= lookahead_) {
      heading = std::atan2(dy, dx);
      return true;
    }
  }
  const auto last = to_costmap_frame(global_plan_.poses.back());
  const double dx = last.pose.position.x - pose.pose.position.x;
  const double dy = last.pose.position.y - pose.pose.position.y;
  if (std::hypot(dx, dy) < 1e-3) {
    return false;
  }
  heading = std::atan2(dy, dx);
  return true;
}

geometry_msgs::msg::TwistStamped LaneController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.header.stamp = clock_->now();

  const double rover_yaw = tf2::getYaw(pose.pose.orientation);
  double desired_heading = rover_yaw;

  double lx, ly, lyaw;
  if (measuredLane(lx, ly, lyaw)) {
    // Steer by the lane itself. Cross-track is measured from the line through
    // the measured centre, less where in the lane we mean to sit.
    const double nx = -std::sin(lyaw), ny = std::cos(lyaw);
    const double cross =
      (pose.pose.position.x - lx) * nx + (pose.pose.position.y - ly) * ny - lane_offset_;

    if (std::abs(cross) < cross_track_deadband_) {
      desired_heading = lyaw;
    } else {
      const double correction =
        std::clamp(-cross_track_kp_ * cross, -max_correction_, max_correction_);
      desired_heading = lyaw + correction;
    }
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 5000, "%s: steering by the measured lane",
                         plugin_name_.c_str());
  } else if (headingFromPlan(pose, desired_heading)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
                         "%s: no lane measurement - following the plan instead",
                         plugin_name_.c_str());
  } else {
    return cmd;   // nothing to steer by and nowhere to go
  }

  // Rotate or translate, never both. Driving while the heading is still
  // changing integrates distance along a heading that is already out of date,
  // and on this rover that costs roughly 70x more position error per metre.
  const double error = wrap(desired_heading - rover_yaw);
  if (std::abs(error) > align_tolerance_) {
    const double floor_rate = std::min(min_pivot_speed_, max_angular_vel_);
    const double rate = std::min(
      max_angular_vel_, std::max(floor_rate, heading_kp_ * std::abs(error)));
    cmd.twist.angular.z = std::copysign(rate, error);
    cmd.twist.linear.x = 0.0;
  } else {
    cmd.twist.angular.z = 0.0;
    cmd.twist.linear.x = desired_linear_vel_ * speed_limit_;
  }
  return cmd;
}

}  // namespace egrobots_lane_controller

PLUGINLIB_EXPORT_CLASS(
  egrobots_lane_controller::LaneController, nav2_core::Controller)
